# robot-cx-22 R9 progress report

Baseline: `dev-phase-1.1-a` at `4b8ba23`.

## Contract audit

- `IDrive`, `IShooter`, `IIntake`, and `ITurret` are narrow mechanism APIs;
  lifecycle is `observe(RobotState)` then `update(RobotAction.Builder)` in
  `ISubsystem` (`TeamCode/core/src/main/java/boobuzz/core/subsystem/*.java`).
- `Subsystems` observes and updates drive, shooter, intake, and turret in a
  fixed order (`Subsystems.java:11-40`). `CplxEngine1` owns motion, automatic
  turret aim, shooter sequencing, and intake ownership
  (`logic/cplx1/CplxEngine1.java:22-91`); `DirectEngine` is the passthrough
  request map (`logic/direct/DirectEngine.java:20-151`).
- HAL-clock behavior is present in `StubShooter` and `StubTurret`: timestamps
  come from `RobotState.t()` (`subsystem/stub/StubShooter.java:20-23`,
  `StubTurret.java:26-30`), and feed/settle thresholds use the compile-time
  constants (`RobotConstants.java:39-41`). Existing tests cover spin-up/feed
  timing, turret settling, and both-engine cancellation paths.

## Actual defect selected for R9

`StubTurret.hold()` only clears `scanning` (`StubTurret.java:58-61`). It leaves
`aiming`, `aimStartedMs`, and pending lock/scan events active. If a turret is
held while still settling, a later HAL timestamp makes `onTarget()` true and
`update()` can emit `turret.locked` after the owner was cancelled. This violates
the R3 automatic-turret hold/cancel boundary described in `request-flow.md:32-37`
and the R5 cancel-all turret safety contract. The implementation will clear the
settling/aim state and pending transition events while preserving the current
angle, with a focused regression test.

## Approved missing behavior (not an R9 defect)

- `StubShooter`, `StubIntake`, and `StubTurret` are explicitly timing-only
  Phase 1.1 mechanisms; real shooter/intake/turret device implementations are
  deferred until Phase 2 (`design-spec.md:81-87`).
- `RobotConstants.SERVOS` is empty and `RealHal.write()` only applies the
  declared motor/servo maps; no new hardware names or request types will be
  invented in this round.

## Hardware-facing gap inventory for the next round

- `Hardware.java:25-66` maps the four wheel motors, declared servos, and a
  hard-coded FTC device named `pinpoint`; it has no shooter, intake, or turret
  device adapter.
- `RealHal.java:43-79` reads wheel encoders/velocity, Pinpoint, voltage, and
  gamepad state, and writes only the mechanism motor/servo maps; it intentionally
  ignores `RobotAction.events` (`RealHal.java:70-79`).
- `AutoMain.java:22-31` and `TeleopMain.java:25-48` construct the shared core
  with `RobotFactory`, so the current production mechanism set remains the
  stubs. Their lifecycle/yield paths are present (`AutoMain.java:37-65`,
  `TeleopMain.java:49-78`); device-backed subsystem wiring is the next
  hardware-facing scope, not an R9 invention.

## Round status

The focused turret hold regression, pymunk test-line checks, final six-auto
matrix, and read-only simulator cross-review will be appended with commit hashes
and evidence as they complete.

## Completed source change

- `1e555cb` — `StubTurret.hold()` now cancels aiming/settling and clears stale
  lock/scan events while preserving the current angle. `StubTurretTest` adds
  `holdCancelsPendingAimSettleAndTransitionEvents`.
- Targeted test: `JAVA_HOME=/usr/lib/jvm/java-21-openjdk ./gradlew :core:test
  --tests boobuzz.core.subsystem.stub.StubTurretTest` — green.
- Pymunk Java smoke (ports 5810/5811, seed 42, 1000 ticks, tap port 0):
  `cplx1` and `direct` both passed; target pose `(120.03,72.02,0.000)` and
  sensor pose `(119.99,71.98,6.282)` in each run.

## Round verification

The source change is pinned at `1e555cb` for the simulator cross-review. The
review is `review-sim-cx-21-robot.md`, against simulator `dev-phase-1.1-a`
`518bf9c` (read-only). It records one major simulator gap (multi-robot socket
read/write deadlines) and two minor gaps (fractional event timestamp
truncation and missing fresh-process/network determinism coverage); none is a
robot-code defect or was silently changed here.

Full verification command:
`JAVA_HOME=/usr/lib/jvm/java-21-openjdk ./gradlew :core:test :sim:test
:TeamCode:assembleDebug --rerun-tasks` — `:core:test` 122 passed, `:sim:test`
10 passed, zero failures/errors/skips, and `:core:sdkGuard`/`:sim:sdkGuard`
passed as part of the build. `:TeamCode:assembleDebug` passed.

Pymunk headless servers were isolated to ports 5812–5819 and stopped after each
run. Target error is final pinpoint versus the sequence's last target; sensor
error is final pinpoint versus the server truth (heading deltas normalized).
Values below use the printed pose precision. Both engines produced the same
endpoint; `ticks` is `cplx1/direct`.

| auto | target pose | cplx1 = direct final pose | target error (in/deg) | sensor error (in/deg) | ticks |
|---|---|---|---:|---:|---:|
| BlueDoggy6Piece | (40.05,16.13,3.142) | (40.13,16.13,3.142) | 0.083 / +0.023 | 0.060 / +0.057 | 969/969 |
| RedDoggy6Piece | (103.95,16.13,0.000) | (104.00,16.08,6.282) | 0.066 / −0.068 | 0.042 / −0.057 | 1088/1088 |
| BlueMissionary9Piece | (55.87,115.95,1.571) | (55.80,115.94,1.569) | 0.075 / −0.103 | 0.070 / −0.115 | 1672/1672 |
| RedMissionary9Piece | (88.13,115.95,1.571) | (88.13,115.95,1.571) | 0.005 / +0.012 | 0.022 / 0.000 | 1544/1544 |
| BlueMissionary9PieceLever | (52.87,79.95,1.571) | (52.91,80.05,1.572) | 0.104 / +0.069 | 0.054 / +0.057 | 2510/2510 |
| RedMissionary9PieceLever | (91.13,79.95,1.571) | (91.11,79.87,1.568) | 0.084 / −0.160 | 0.060 / −0.115 | 2341/2341 |

`test-line` also passed on both engines (4,000 ticks, port 5812 reused): final
`(120.01,71.98,6.282)`, truth `(119.99,71.98,6.282)`, target error `0.022 in /
−0.057°`, sensor error `0.020 in / 0.000°`; no guard or server failure occurred.

## Completed round

- `1e555cb` — source behavior and focused regression, pushed to
  `origin/dev-phase-1.1-a`.
- `review-sim-cx-21-robot.md` and this report are the R9 audit/cross-review
  artifacts. No hardware names, request types, protocol fields, or simulator
  files were invented or changed.
