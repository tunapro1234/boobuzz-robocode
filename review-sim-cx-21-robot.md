# Simulator cross-review for robot-cx-22 (R9)

Reviewed `re-cock-nize` branch `dev-phase-1.1-a` at `518bf9c` read-only,
including the simulator changes through that hash. Compared the server, bag
reader, tap reader, multi-robot path, and tests with
`docs/phases/phase-1.1/design-spec.md`, `docs/protokol.md`, and the Java
`SimHal`/`SimMain` client. No files in the simulator repository were changed.

## Findings

### [major] Multi-robot socket I/O has no bounded read/write deadline

Location: `re-cock-nize/sim/server.py:376-416`, especially `makefile()` at
line 378 and `sendall()` at line 402. The single-client path applies
`io_timeout_s`, but `_serve_multi_connection` does not set a socket timeout.
A connected robot that sends a partial JSON line, or a client that stops
reading replies, can leave its worker blocked indefinitely. The lockstep
deadline at `server.py:453-524` only runs after another client submits a
complete step; it cannot release an idle partial-line reader or a blocked
writer. This violates the protocol's bounded failure/stop expectation and
can leave the Java `SimHal` exchange hung.

Concrete reproducer: start `--robots 2`, reset both clients, send only
`{"type":"step"` (without a newline) on robot 0, and send no more bytes;
the server thread remains alive beyond `--io-timeout-ms`. A second reproducer
submits valid steps while robot 0 never reads; the `sendall` path has no
deadline. Set the configured timeout on every multi socket (or use explicit
non-blocking deadlines), turn expiry into a clear connection error, and add
partial-line and non-reading-client tests. Keep the lockstep timeout as a
separate tick deadline.

### [minor] Event timestamps silently truncate fractional milliseconds

Location: `re-cock-nize/sim/server.py:779-819`. `_parse_events` accepts any
finite JSON number for `t_ms`, then stores `int(t_raw)` at line 819. A value
such as `1.9` becomes `1`, although the protocol field is a Java millisecond
integer. This creates a timing/logging mismatch rather than rejecting a bad
frame. Require a non-boolean integer (or reject non-integral finite numbers)
and add a fractional timestamp validation test.

### [minor] Fresh-process/network determinism coverage is still absent

Locations: `re-cock-nize/tests/test_determinism.py:37-108` and
`tests/test_multi_robot.py:210-262`. The determinism tests use one in-process
`SimServer` and compare parsed/canonical state values; the two-robot test
compares backend tuples directly. They do not start a fresh server process,
compare raw serialized ready/reset/state lines, or run two actual socket
clients through a process restart/staggered start. This leaves transport,
JSON serialization, and the multi-robot reset/start boundary untested even
though the in-process physics claim is covered. Add a subprocess harness that
runs the same seeded transcript (including ready/reset and two-robot
lockstep) twice and compares the emitted JSONL bytes; keep transport timing
assertions separate from physics equality.

## Verified coverage and conclusion

The reviewed commits cover the earlier input validation, lockstep eviction,
single-client I/O timeout, reset epochs, seam-bag shape, tap-drop metadata,
numeric events, tap reconnect, and servo rejection findings. I found no
additional protocol field-name mismatch on the normal Java one-client path,
and no evidence that seeded physics itself is nondeterministic. The three
findings above are simulator-side follow-ups; this robot round does not edit
the simulator.
