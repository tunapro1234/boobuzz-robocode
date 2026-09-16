#!/usr/bin/env python3
"""Pretty-print JSONL records from the robot debug tap."""

from __future__ import annotations

import argparse
import json
import socket
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host")
    parser.add_argument("port", type=int)
    parser.add_argument("--seam", choices=("hal", "subsystem", "logic"))
    parser.add_argument("--grep", help="only print lines containing this text")
    args = parser.parse_args()

    try:
        with socket.create_connection((args.host, args.port), timeout=5.0) as sock:
            with sock.makefile("r", encoding="utf-8") as stream:
                for raw in stream:
                    line = raw.rstrip("\n")
                    if args.grep and args.grep not in line:
                        continue
                    try:
                        record = json.loads(line)
                    except json.JSONDecodeError:
                        print(line)
                        continue
                    if args.seam and record.get("seam") != args.seam:
                        continue
                    print(json.dumps(record, indent=2, sort_keys=True))
    except KeyboardInterrupt:
        return 0
    except OSError as exc:
        print(f"tap connection failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
