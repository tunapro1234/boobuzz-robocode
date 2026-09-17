#!/usr/bin/env python3
"""Chapter A acceptance runner.

The runner owns only the headless simulator children it starts.  Java owns the
TCP clock and the RobotLoop scenario; this file discovers the sibling simulator,
chooses ports, collects the Java JSONL records, and applies the small A03 gate.
It intentionally never uses the debug SocketController.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time
import re
from typing import Any, Iterable


REQUIRED_FIELDS = {
    "scenario",
    "seed",
    "t_ms",
    "engine",
    "request_statuses",
    "commanded_target",
    "truth",
    "sensed_pose",
    "motors",
    "servos",
    "events",
}
SCENARIOS = ("A-drive", "A-cancel")
DEFAULT_TIMEOUT_SECONDS = 30.0
PORT_MINIMUM = 5580


class RunnerError(RuntimeError):
    """A deterministic runner setup, process, or acceptance failure."""


def repository_root() -> Path:
    return Path(__file__).resolve().parents[1]


def existing_directory(value: str | None, fallback: Path, label: str) -> Path:
    path = Path(value).expanduser().resolve() if value else fallback.resolve()
    if not path.is_dir():
        raise RunnerError(f"{label} does not exist: {path}")
    return path


def discover_simulator(root: Path, explicit: str | None) -> Path:
    configured = explicit or os.environ.get("FTC_SIM_ROOT")
    return existing_directory(configured, root.parent / "re-cock-nize", "simulator root")


def discover_python(simulator: Path, explicit: str | None) -> Path:
    configured = explicit or os.environ.get("PYTHON")
    candidate = Path(configured).expanduser() if configured else simulator / ".venv/bin/python"
    if not candidate.is_file():
        raise RunnerError(
            f"simulator Python executable is missing: {candidate}; "
            "pass --python or set PYTHON"
        )
    # Keep the venv symlink itself.  Resolving it can silently switch from the
    # simulator's dependency environment to the system interpreter (and lose
    # Pymunk), even though the configured executable exists and is runnable.
    return candidate.absolute()


def discover_java(explicit: str | None) -> Path:
    configured = explicit or os.environ.get("JAVA")
    if configured:
        candidate = Path(configured).expanduser()
        if not candidate.is_file():
            located = shutil.which(configured)
            candidate = Path(located) if located else candidate
        if not candidate.is_file():
            raise RunnerError(
                f"Java executable is missing: {candidate}; pass --java or set JAVA_HOME"
            )
        return candidate.resolve()

    candidates: list[Path] = []
    java_home = os.environ.get("JAVA_HOME")
    if java_home:
        candidates.append(Path(java_home) / "bin/java")
    on_path = shutil.which("java")
    if on_path:
        candidates.append(Path(on_path))
    # If PATH points at an older JDK, inspect installed JDK homes rather than
    # silently launching a runtime too old for the compiled Java distribution.
    jvm_root = Path("/usr/lib/jvm")
    if jvm_root.is_dir():
        candidates.extend(sorted(jvm_root.glob("*/bin/java"), reverse=True))

    compatible: list[tuple[int, Path]] = []
    seen: set[Path] = set()
    for candidate in candidates:
        if not candidate.is_file():
            continue
        candidate = candidate.resolve()
        if candidate in seen:
            continue
        seen.add(candidate)
        try:
            version = subprocess.run(
                [str(candidate), "-version"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=2.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            continue
        match = re.search(r'version\s+"(\d+)', version.stderr + version.stdout)
        if match and int(match.group(1)) >= 17:
            compatible.append((int(match.group(1)), candidate))
    if compatible:
        return max(compatible, key=lambda item: item[0])[1]
    raise RunnerError(
        "no installed Java runtime >= 17 was found; pass --java or set JAVA_HOME"
    )


def discover_classpath(root: Path, explicit_dist: str | None,
                       explicit_classpath: str | None) -> str:
    if explicit_classpath:
        return explicit_classpath
    configured = explicit_dist or os.environ.get("FTC_SIM_DIST")
    dist = Path(configured).expanduser() if configured else root / "sim/build/install/sim"
    lib = dist / "lib"
    if not lib.is_dir():
        raise RunnerError(
            f"installed Java distribution is missing: {lib}; run "
            "JAVA_HOME=/usr/lib/jvm/java-21-openjdk ./gradlew :sim:installDist "
            "or pass --classpath"
        )
    return str(lib / "*")


def free_port(minimum: int = PORT_MINIMUM) -> int:
    for port in range(minimum, 65536):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(("127.0.0.1", port))
            except OSError:
                continue
            return port
    raise RunnerError("no free localhost port")


def wait_for_port(process: subprocess.Popen[str], port: int,
                  timeout: float = 10.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RunnerError(f"simulator exited before listening on {port}")
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.15):
                return
        except OSError:
            time.sleep(0.025)
    raise RunnerError(f"simulator did not listen on {port} within {timeout:.1f}s")


def stop_owned_process(process: subprocess.Popen[str] | None) -> None:
    """Stop exactly one process started by this runner; never use a process glob."""
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)


def start_server(simulator: Path, python: Path, mechanism: Path, physics: str,
                 port: int, log_path: Path) -> subprocess.Popen[str]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = log_path.open("w", encoding="utf-8")
    command = [
        str(python), "-m", "sim.server",
        "--mechanism", str(mechanism),
        "--physics", physics,
        "--headless", "--port", str(port),
    ]
    try:
        process = subprocess.Popen(
            command,
            cwd=simulator,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
        )
    finally:
        # The child has its own file descriptor; closing the runner's copy avoids
        # retaining a descriptor across the many fresh-process repetitions.
        log.close()
    try:
        wait_for_port(process, port)
    except Exception:
        stop_owned_process(process)
        raise
    return process


def java_records(command: list[str], timeout: float) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    try:
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        raise RunnerError(f"Java acceptance process exceeded {timeout:.1f}s") from error

    # Preserve the Java trace on runner stdout as well as in the scenario file.
    if completed.stdout:
        sys.stdout.write(completed.stdout)
        sys.stdout.flush()
    if completed.stderr:
        sys.stderr.write(completed.stderr)
        sys.stderr.flush()

    records: list[dict[str, Any]] = []
    result: dict[str, Any] | None = None
    for line in completed.stdout.splitlines():
        if line.startswith("ACCEPTANCE_RESULT "):
            try:
                result = json.loads(line[len("ACCEPTANCE_RESULT "):])
            except json.JSONDecodeError as error:
                raise RunnerError(f"invalid Java acceptance result: {line}") from error
            continue
        if not line.startswith("{"):
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as error:
            raise RunnerError(f"invalid Java trace JSON: {line[:200]}") from error
        if not isinstance(value, dict):
            raise RunnerError("Java trace line is not an object")
        records.append(value)
    if completed.returncode != 0:
        reason = result.get("failures") if result else completed.stderr.strip()
        raise RunnerError(
            f"Java acceptance process failed ({completed.returncode}): {reason}"
        )
    if result is None or result.get("ok") is not True:
        raise RunnerError(f"Java acceptance result failed: {result}")
    return records, result


def validate_record(record: dict[str, Any], scenario: str, seed: int) -> None:
    missing = REQUIRED_FIELDS - set(record)
    if missing:
        raise RunnerError(f"{scenario} seed {seed} trace missing fields: {sorted(missing)}")
    if record["scenario"] != scenario or int(record["seed"]) != seed:
        raise RunnerError(f"trace identity mismatch: {record}")
    if not isinstance(record["request_statuses"], list):
        raise RunnerError("request_statuses must be a list")
    for key in ("motors", "servos"):
        if not isinstance(record[key], dict):
            raise RunnerError(f"{key} must be an object")
    if not isinstance(record["events"], list):
        raise RunnerError("events must be a list")


def without_wall_diagnostics(value: Any) -> Any:
    if isinstance(value, dict):
        return {
            key: without_wall_diagnostics(item)
            for key, item in value.items()
            if key not in {"wall_ms", "wall_time_ms", "elapsed_ms"}
        }
    if isinstance(value, list):
        return [without_wall_diagnostics(item) for item in value]
    return value


def assert_deterministic(first: list[dict[str, Any]], second: list[dict[str, Any]],
                         label: str) -> None:
    left = without_wall_diagnostics(first)
    right = without_wall_diagnostics(second)
    if left != right:
        for index, (lhs, rhs) in enumerate(zip(left, right)):
            if lhs != rhs:
                raise RunnerError(f"{label} transcript differs at tick {index}")
        raise RunnerError(f"{label} transcript lengths differ")


def pose_error(record: dict[str, Any]) -> tuple[float | None, float | None]:
    target = record.get("commanded_target")
    truth = record.get("truth")
    sensed = record.get("sensed_pose")
    if not isinstance(target, dict) or not isinstance(truth, dict):
        return None, None
    dx = float(truth["x"]) - float(target["x"])
    dy = float(truth["y"]) - float(target["y"])
    distance = (dx * dx + dy * dy) ** 0.5
    sensor_error = None
    if isinstance(sensed, dict):
        sx = float(sensed["x"]) - float(truth["x"])
        sy = float(sensed["y"]) - float(truth["y"])
        sensor_error = (sx * sx + sy * sy) ** 0.5
    return distance, sensor_error


def run_one(root: Path, simulator: Path, python: Path, java: Path, classpath: str,
            mechanism: Path, output_dir: Path, scenario: str, engine: str,
            seed: int, ticks: int, dt_ms: int, physics: str,
            run_number: int, path_id: str, start_pose: dict[str, Any],
            target_pose: dict[str, Any]) -> list[dict[str, Any]]:
    port = free_port()
    server: subprocess.Popen[str] | None = None
    log_path = output_dir / "server-logs" / (
        f"{scenario}-{engine}-seed{seed}-run{run_number}-port{port}.log"
    )
    try:
        server = start_server(simulator, python, mechanism, physics, port, log_path)
        command = [
            str(java), "-cp", classpath, "boobuzz.sim.AcceptanceMain",
            "--scenario", scenario,
            "--engine", engine,
            "--seed", str(seed),
            "--host", "127.0.0.1",
            "--port", str(port),
            "--dt", str(dt_ms),
            "--ticks", str(ticks),
            "--path", path_id,
            "--start-x", str(start_pose["x"]),
            "--start-y", str(start_pose["y"]),
            "--start-h", str(start_pose["h"]),
            "--target-x", str(target_pose["x"]),
            "--target-y", str(target_pose["y"]),
            "--target-h", str(target_pose["h"]),
        ]
        records, result = java_records(command, DEFAULT_TIMEOUT_SECONDS)
        for record in records:
            validate_record(record, scenario, seed)
        if len(records) != ticks:
            raise RunnerError(
                f"{scenario} {engine} seed {seed}: expected {ticks} records, "
                f"got {len(records)}"
            )
        errors = pose_error(records[-1]) if records else (None, None)
        print(
            f"{scenario} engine={engine} seed={seed} run={run_number}: "
            f"ticks={result.get('ticks')} target_error_in={errors[0]} "
            f"sensor_error_in={errors[1]}"
        )
        return records
    finally:
        stop_owned_process(server)


def parse_seeds(value: str) -> list[int]:
    try:
        seeds = [int(part.strip()) for part in value.split(",") if part.strip()]
    except ValueError as error:
        raise argparse.ArgumentTypeError("seeds must be comma-separated integers") from error
    if not seeds:
        raise argparse.ArgumentTypeError("at least one seed is required")
    return seeds


def scenario_config(fixture: dict[str, Any], name: str) -> dict[str, Any]:
    try:
        value = fixture["scenarios"][name]
    except (KeyError, TypeError) as error:
        raise RunnerError(f"fixture has no {name} scenario") from error
    if not isinstance(value, dict):
        raise RunnerError(f"fixture scenario {name} is not an object")
    return value


def write_trace(path: Path, records: Iterable[dict[str, Any]], mode: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open(mode, encoding="utf-8") as output:
        for record in records:
            output.write(json.dumps(record, separators=(",", ":"), sort_keys=True))
            output.write("\n")


def run(args: argparse.Namespace) -> int:
    root = repository_root()
    if args.chapter != "A":
        raise RunnerError("this runner currently implements only --chapter A")
    if args.physics != "pymunk":
        raise RunnerError("A03 gates only --physics pymunk")
    simulator = discover_simulator(root, args.sim_root)
    python = discover_python(simulator, args.python)
    java = discover_java(args.java)
    classpath = discover_classpath(root, args.sim_dist, args.classpath)
    mechanism = Path(args.mechanism).expanduser().resolve() if args.mechanism else root / (
        "TeamCode/core/src/main/java/boobuzz/core/hal/RobotConstants.java"
    )
    if not mechanism.is_file():
        raise RunnerError(f"mechanism constants file is missing: {mechanism}")
    fixture_path = Path(args.fixture).expanduser().resolve() if args.fixture else root / (
        "tools/fixtures/phase11a-A.json"
    )
    try:
        fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RunnerError(f"could not read fixture {fixture_path}: {error}") from error
    dt_ms = int(fixture.get("dt_ms", 20))
    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else root / (
        "build/acceptance-A"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    seeds = args.seeds
    start_pose = fixture.get("start_pose", {"x": 72.0, "y": 72.0, "h": 0.0})
    target_pose = fixture.get("target_pose", {"x": 120.0, "y": 72.0, "h": 0.0})
    if not isinstance(start_pose, dict) or not isinstance(target_pose, dict):
        raise RunnerError("fixture start_pose and target_pose must be objects")

    drive = scenario_config(fixture, "A-drive")
    drive_ticks = int(drive.get("ticks", 1000))
    engines = [str(engine) for engine in drive.get("engines", ["direct", "cplx1"])]
    drive_trace = output_dir / "A-drive.jsonl"
    drive_trace.unlink(missing_ok=True)
    first_seed1: dict[str, list[dict[str, Any]]] = {}
    for engine in engines:
        for seed in seeds:
            repeats = int(drive.get("seed1_repeats", 2)) if seed == 1 else int(
                drive.get("seed42_repeats", 1)
            )
            for run_number in range(repeats):
                records = run_one(
                    root, simulator, python, java, classpath, mechanism, output_dir,
                    "A-drive", engine, seed, drive_ticks, dt_ms, args.physics, run_number,
                    str(drive.get("path", "test-line")), start_pose, target_pose,
                )
                write_trace(drive_trace, records, "a")
                if seed == 1:
                    previous = first_seed1.setdefault(engine, records)
                    if run_number > 0:
                        assert_deterministic(previous, records,
                                             f"A-drive {engine} seed1")

    cancel = scenario_config(fixture, "A-cancel")
    cancel_ticks = int(cancel.get("ticks", 101))
    cancel_trace = output_dir / "A-cancel.jsonl"
    cancel_trace.unlink(missing_ok=True)
    requested_cancel_seeds = [seed for seed in (42, 1) if seed in seeds]
    if not requested_cancel_seeds:
        requested_cancel_seeds = seeds
    for run_number, seed in enumerate(requested_cancel_seeds):
        records = run_one(
            root, simulator, python, java, classpath, mechanism, output_dir,
            "A-cancel", "direct", seed, cancel_ticks, dt_ms, args.physics, run_number,
            str(cancel.get("path", "test-line")), start_pose, target_pose,
        )
        write_trace(cancel_trace, records, "a")

    print(f"A03 traces: {drive_trace} and {cancel_trace}")
    return 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--chapter", default="A")
    result.add_argument("--physics", default="pymunk")
    result.add_argument("--seeds", type=parse_seeds, default=[1, 42])
    result.add_argument("--output-dir", "--output", dest="output_dir")
    result.add_argument("--sim-root")
    result.add_argument("--python")
    result.add_argument("--java")
    result.add_argument("--sim-dist")
    result.add_argument("--classpath")
    result.add_argument("--mechanism")
    result.add_argument("--fixture")
    return result


def main() -> int:
    try:
        return run(parser().parse_args())
    except RunnerError as error:
        print(f"acceptance: ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
