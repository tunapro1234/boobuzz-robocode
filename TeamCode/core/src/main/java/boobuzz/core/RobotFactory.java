package boobuzz.core;

import boobuzz.core.controller.Controller;
import boobuzz.core.controller.GamepadController;
import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Intent;
import boobuzz.core.hal.Hal;
import boobuzz.core.logic.cplx_engine_1.CplxEngine1;
import boobuzz.core.hal.Mechanism;

import java.util.Objects;

/** Builds the shared engine/controller chain for the robot and simulator. */
public final class RobotFactory {

    private RobotFactory() {}

    public static RobotLoop create(Hal hal, Mechanism mechanism) {
        return create(hal, mechanism, null);
    }

    /** When {@code fixedDrive == null}, the controller uses HAL's gamepad source. */
    public static RobotLoop create(Hal hal, Mechanism mechanism, Drive fixedDrive) {
        Objects.requireNonNull(hal, "hal");
        Objects.requireNonNull(mechanism, "mechanism");

        Controller controller = fixedDrive == null
                ? new GamepadController(hal)
                : feedback -> Intent.of(fixedDrive);
        return new RobotLoop(hal, new CplxEngine1(mechanism), controller);
    }
}
