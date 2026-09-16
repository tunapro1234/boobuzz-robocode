# `:core` architecture

`:core` is the SDK-free robot brain.  The real robot and the simulator compile
and run the same controller and logic bytecode; only the HAL/bootstrap differs.

```text
boobuzz/core/
  contract/   Request, RequestType, RequestStatus, PathRequest, RequestStream,
              RequestBatch, Feedback, RobotState, RobotAction, Event,
              WorldSnapshot, GamepadState, IGamepadSource
  hal/        IHal, Mechanism, RobotConstants
  subsystem/  ISubsystem, IDrive, IShooter, IIntake, ITurret, Subsystems
    pedro/    PedroDrive, HalDrivetrain, HalLocalizer, PathRegistry, PedroConstants
    stub/     StubShooter, StubIntake, StubTurret
  logic/      IRobotEngine
    direct/   DirectEngine, DirectMap
    cplx1/    CplxEngine1, MotionLogic, TurretLogic, ShooterLogic
  controller/ IController, Buttons
    teleop/   TeleopController, TeleopMap
    auto/     AutoController, SequenceRunner, AutoBuilder, AutoSequence, AutoStep
    opmodes/  AutoRegistry, AutoLocations, six autonomous routines
  RobotLoop, RobotFactory
```

## Add a subsystem

1. Define the smallest command/observation interface in `subsystem/` (usually an
   `IThing extends ISubsystem`).  Keep it independent of controllers and SDK types.
2. Add `subsystem/stub/StubThing` with deterministic timing and focused unit tests.
3. Add the instance to `Subsystems`, its fixed observe/update order, and `RobotFactory`.
4. Add one request mapping case in `logic/direct/DirectMap` (and a cplx1 logic owner
   only when the behavior is genuinely intelligent).  Do not duplicate mechanism
   code in `sim/` or let a controller call a subsystem directly.

## Add a request type

1. Add the edge-triggered enum member and a typed helper to `contract/Request`.
2. Add exactly one type-specific case to `DirectMap`; report completion or rejection
   through `RequestStatus`.  Route the same type through the owning cplx1 logic when
   it needs policy; stream-level drive remains in `RequestStream`.
3. Add controller/sequence mapping, focused engine tests, and an end-to-end test.
   Update the protocol documentation if the wire payload changes.  `WAIT` remains a
   controller-local timer, while `SWITCH_ENGINE` is handled by `RobotLoop`.

Keep contract records as data-only DTOs, preserve HAL/logic/controller boundaries,
and run `:core:test :sim:test :TeamCode:assembleDebug` before pushing.
