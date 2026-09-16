package boobuzz.core.hal;

/**
 * Single-frame gamepad reading, corresponding to the {@code gamepad} field in the protocol documentation.
 *
 * <p>In the simulator pygame fills it from the keyboard/joystick; on the robot it
 * comes from the OpMode's {@code gamepad1}. :core does not know which one (rule 3).
 *
 * <p>Stick values are -1..1 and triggers are 0..1. {@code ly} is NEGATIVE WHEN
 * PUSHED UP (both pygame and FTC report this); use {@code -ly} for forward.
 */
public record GamepadState(double lx, double ly, double rx, double ry,
                           boolean a, boolean b, boolean x, boolean y,
                           boolean lb, boolean rb,
                           double lt, double rt,
                           Dpad dpad) {

    public enum Dpad { NONE, UP, DOWN, LEFT, RIGHT }

    public static GamepadState neutral() {
        return new GamepadState(0, 0, 0, 0,
                false, false, false, false,
                false, false,
                0, 0,
                Dpad.NONE);
    }
}
