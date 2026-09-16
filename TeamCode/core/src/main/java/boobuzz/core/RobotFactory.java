package boobuzz.core;

import boobuzz.core.controller.Controller;
import boobuzz.core.controller.GamepadController;
import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Intent;
import boobuzz.core.hal.Hal;
import boobuzz.core.logic.cplx_engine_1.CplxEngine1;
import boobuzz.core.logic.direct_engine.DirectEngine;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.subsystem.StubIntake;
import boobuzz.core.subsystem.StubShooter;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.pedro.PedroDrive;

import java.util.Objects;

/** Builds the shared engine/controller chain for the robot and simulator. */
public final class RobotFactory {

    private RobotFactory() {}

    public static RobotLoop create(Hal hal, Mechanism mechanism) {
        return create(hal, mechanism, "cplx_engine_1", null);
    }

    /** When {@code fixedDrive == null}, the controller uses HAL's gamepad source. */
    public static RobotLoop create(Hal hal, Mechanism mechanism, Drive fixedDrive) {
        return create(hal, mechanism, "cplx_engine_1", fixedDrive);
    }

    public static RobotLoop create(Hal hal, Mechanism mechanism, String engineName) {
        return create(hal, mechanism, engineName, null);
    }

    /** Builds the selected engine over one fixed-order subsystem set. */
    public static RobotLoop create(Hal hal, Mechanism mechanism,
                                   String engineName, Drive fixedDrive) {
        Controller controller = fixedDrive == null
                ? new GamepadController(hal)
                : feedback -> Intent.of(fixedDrive);
        return createWithController(hal, mechanism, engineName, controller);
    }

    /** Builds the selected engine with an explicit controller, used by autonomous entry points. */
    public static RobotLoop createWithController(Hal hal, Mechanism mechanism,
                                                  String engineName, Controller controller) {
        Objects.requireNonNull(hal, "hal");
        Objects.requireNonNull(mechanism, "mechanism");
        Objects.requireNonNull(engineName, "engineName");
        Objects.requireNonNull(controller, "controller");
        Subsystems subsystems = new Subsystems(
                new PedroDrive(mechanism), new StubShooter(), new StubIntake());
        var engine = switch (engineName) {
            case "cplx_engine_1" -> new CplxEngine1(subsystems);
            case "direct" -> new DirectEngine(subsystems);
            default -> throw new IllegalArgumentException(
                    "unknown engine: " + engineName + " (expected direct or cplx_engine_1)");
        };
        return new RobotLoop(hal, engine, controller);
    }
}
