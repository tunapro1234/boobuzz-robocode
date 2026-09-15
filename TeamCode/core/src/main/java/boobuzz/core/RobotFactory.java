package boobuzz.core;

import boobuzz.core.controller.Controller;
import boobuzz.core.controller.GamepadController;
import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Intent;
import boobuzz.core.hal.Hal;
import boobuzz.core.logic.cplx_engine_1.CplxEngine1;
import boobuzz.core.hal.Mechanism;

import java.util.Objects;

/** Robot ve sim icin ortak engine/controller zincirini kurar. */
public final class RobotFactory {

    private RobotFactory() {}

    public static RobotLoop create(Hal hal, Mechanism mechanism) {
        return create(hal, mechanism, null);
    }

    /** {@code fixedDrive == null} ise controller HAL'in gamepad kaynagini kullanir. */
    public static RobotLoop create(Hal hal, Mechanism mechanism, Drive fixedDrive) {
        Objects.requireNonNull(hal, "hal");
        Objects.requireNonNull(mechanism, "mechanism");

        Controller controller = fixedDrive == null
                ? new GamepadController(hal)
                : feedback -> Intent.of(fixedDrive);
        return new RobotLoop(hal, new CplxEngine1(mechanism), controller);
    }
}
