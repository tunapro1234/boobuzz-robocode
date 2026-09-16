#!/usr/bin/env python3
"""Send JSONL RequestBatch commands to a SocketController smoke endpoint."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import threading


def feedback_reader(stream) -> None:
    try:
        for line in stream:
            try:
                record = json.loads(line)
                print("feedback:", json.dumps(record, sort_keys=True), flush=True)
            except json.JSONDecodeError:
                print("feedback:", line.rstrip(), flush=True)
    except OSError:
        pass


def command(line: str) -> str:
    """Return a validated batch, accepting JSON or ``vx vy omega`` shorthand."""
    text = line.strip()
    if not text:
        return ""
    if text.startswith("{"):
        value = json.loads(text)
        if not isinstance(value, dict):
            raise ValueError("a command must be a JSON object")
        return json.dumps(value, separators=(",", ":"))
    parts = text.replace(",", " ").split()
    if len(parts) != 3:
        raise ValueError("use a RequestBatch JSON object or vx vy omega")
    vx, vy, omega = (float(part) for part in parts)
    return json.dumps({
        "stream": {"vx": vx, "vy": vy, "omega": omega, "manualDrive": True},
        "requests": [],
        "cancels": [],
    }, separators=(",", ":"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host")
    parser.add_argument("port", type=int)
    args = parser.parse_args()

    try:
        with socket.create_connection((args.host, args.port), timeout=5.0) as sock:
            reader = sock.makefile("r", encoding="utf-8")
            thread = threading.Thread(target=feedback_reader, args=(reader,), daemon=True)
            thread.start()
            for line in sys.stdin:
                try:
                    payload = command(line)
                except (ValueError, json.JSONDecodeError) as exc:
                    print(f"invalid command: {exc}", file=sys.stderr)
                    continue
                if payload:
                    sock.sendall((payload + "\n").encode("utf-8"))
    except KeyboardInterrupt:
        return 0
    except OSError as exc:
        print(f"drive connection failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
