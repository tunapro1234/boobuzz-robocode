package boobuzz.core;

import boobuzz.core.controller.Controller;
import boobuzz.core.controller.GamepadController;
import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Intent;
import boobuzz.core.hal.Hal;
import boobuzz.core.logic.engine.C1DriveEngine;
import boobuzz.core.logic.engine.PedroDriveEngine;
import boobuzz.core.logic.engine.RobotEngine;
import boobuzz.core.mechanism.Mechanism;

import java.util.Objects;

/** Robot ve sim icin engine/controller secimini tek yerde kurar. */
public final class RobotFactory {

    public enum EngineKind { C1, PEDRO }

    private RobotFactory() {}

    public static RobotLoop create(Hal hal, Mechanism mechanism, EngineKind engineKind) {
        return create(hal, mechanism, engineKind, null);
    }

    /** {@code fixedDrive == null} ise controller HAL'in gamepad kaynagini kullanir. */
    public static RobotLoop create(Hal hal, Mechanism mechanism, EngineKind engineKind,
                                   Drive fixedDrive) {
        Objects.requireNonNull(hal, "hal");
        Objects.requireNonNull(mechanism, "mechanism");
        Objects.requireNonNull(engineKind, "engineKind");

        RobotEngine engine = switch (engineKind) {
            case C1 -> new C1DriveEngine(mechanism);
            case PEDRO -> new PedroDriveEngine(mechanism);
        };
        Controller controller = fixedDrive == null
                ? new GamepadController(hal)
                : feedback -> Intent.of(fixedDrive);
        return new RobotLoop(hal, engine, controller);
    }
}
