#!/usr/bin/env python3
"""Closed-loop SocketController example: drive test-line, then shoot three."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from typing import Any


def empty_batch() -> dict[str, Any]:
    return {
        "stream": {"vx": 0.0, "vy": 0.0, "omega": 0.0, "manualDrive": False},
        "requests": [],
        "cancels": [],
    }


def request_batch(request_id: int, request_type: str, params: list[float]) -> dict[str, Any]:
    batch = empty_batch()
    batch["requests"] = [{
        "id": request_id,
        "type": request_type,
        "params": params,
        "path": None,
    }]
    return batch


class AgentError(RuntimeError):
    """A controller connection or request failure."""


class AgentConnection:
    def __init__(self, host: str, port: int, connect_timeout: float) -> None:
        self.socket = socket.create_connection((host, port), timeout=connect_timeout)
        self.buffer = bytearray()

    def send(self, batch: dict[str, Any]) -> None:
        line = json.dumps(batch, separators=(",", ":"), allow_nan=False) + "\n"
        self.socket.sendall(line.encode("utf-8"))

    def read_line(self, deadline: float) -> str | None:
        while True:
            newline = self.buffer.find(b"\n")
            if newline >= 0:
                line = bytes(self.buffer[:newline])
                del self.buffer[: newline + 1]
                return line.decode("utf-8")
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            self.socket.settimeout(min(0.2, remaining))
            try:
                chunk = self.socket.recv(4096)
            except socket.timeout:
                return None
            if not chunk:
                raise AgentError("SocketController closed the connection")
            self.buffer.extend(chunk)

    def close(self) -> None:
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.socket.close()


def wait_for_done(connection: AgentConnection, request_id: int,
                  request: dict[str, Any], timeout: float, refresh: float) -> None:
    deadline = time.monotonic() + timeout
    next_refresh = time.monotonic() + refresh
    while True:
        now = time.monotonic()
        if now >= deadline:
            raise AgentError(f"timeout waiting for request {request_id} DONE")
        line = connection.read_line(min(deadline, now + 0.2))
        if line:
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise AgentError(f"invalid feedback JSON: {exc}") from exc
            if record.get("type") != "feedback":
                continue
            feedback = record.get("feedback")
            if not isinstance(feedback, dict):
                continue
            statuses = feedback.get("statuses", [])
            if not isinstance(statuses, list):
                continue
            for status in statuses:
                if not isinstance(status, dict) or status.get("id") != request_id:
                    continue
                state = status.get("state")
                if state == "DONE":
                    return
                if state in {"REJECTED", "FAILED"}:
                    note = status.get("note", "no reason")
                    if note in {"drive already has a request",
                                 "shooter already has a request"}:
                        # SocketController retains the last edge batch until the
                        # next line arrives; duplicate delivery is harmless while
                        # the keepalive empty batch is clearing it.
                        continue
                    raise AgentError(
                        f"request {request_id} {state}: {note}")
        if time.monotonic() >= next_refresh:
            # Keep the watchdog alive without replaying the edge-triggered request.
            connection.send(empty_batch())
            next_refresh = time.monotonic() + refresh


def run(args: argparse.Namespace) -> int:
    connection: AgentConnection | None = None
    try:
        connection = AgentConnection(args.host, args.port, args.connect_timeout)
        goto = request_batch(1, "GOTO", [120.0, 72.0, 0.0])
        connection.send(goto)
        wait_for_done(connection, 1, goto, args.timeout, args.refresh)

        shoot = request_batch(2, "SHOOT", [3.0])
        connection.send(shoot)
        wait_for_done(connection, 2, shoot, args.timeout, args.refresh)

        connection.send(empty_batch())
        return 0
    except (AgentError, OSError, ValueError) as exc:
        print(f"agent_example: {exc}", file=sys.stderr)
        return 1
    finally:
        if connection is not None:
            connection.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5601)
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="seconds allowed for each request")
    parser.add_argument("--refresh", type=float, default=0.1,
                        help="seconds between watchdog keepalive batches")
    parser.add_argument("--connect-timeout", type=float, default=5.0)
    args = parser.parse_args()
    if args.timeout <= 0.0 or args.refresh <= 0.0 or args.connect_timeout <= 0.0:
        parser.error("timeouts and refresh must be positive")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
