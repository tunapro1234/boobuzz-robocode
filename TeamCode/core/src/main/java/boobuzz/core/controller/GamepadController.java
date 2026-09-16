package boobuzz.core.controller;

import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.hal.GamepadSource;
import boobuzz.core.hal.GamepadState;

import com.pedropathing.math.Pose;

/**
 * L3 - converts driver input into intent.
 *
 * <p>The left stick drives and the right stick X controls rotation; stick axes
 * are inverted (pushing up gives a negative value).
 *
 * <p>The deadband belongs here, not in HAL: gamepad hardware does not return
 * exactly to center, and this is the same problem on both sides (simulator, robot).
 *
 * <p><b>Field-oriented:</b> ON by default. The left stick is read in the field
 * frame (up = field {@code +x}), rotated back by heading, and converted to the
 * robot frame. The {@link Drive} type and engine do not know about this; conversion
 * ends entirely in L3, and the downward value remains robot-frame {@code Drive.Manual}.
 *
 * <p>{@code b} toggles the mode and {@code y} resets heading. Resetting does not
 * touch HAL; it is an offset held here (constitution rule 3: :core does not know
 * which hardware is present and does not recalibrate the sensor).
 */
public final class GamepadController implements Controller {

    private static final double DEADBAND = 0.05;

    private final GamepadSource gamepads;

    private boolean fieldOriented = true;
    private double headingOffset = 0.0;
    private boolean prevToggle;
    private boolean prevReset;

    public GamepadController(GamepadSource gamepads) {
        this.gamepads = gamepads;
    }

    @Override
    public Intent decide(Feedback feedback) {
        GamepadState g = gamepads.get();
        if (g == null) {
            return Intent.idle();
        }

        // Edge trigger: holding a button must not trigger it every tick.
        if (g.b() && !prevToggle) {
            fieldOriented = !fieldOriented;
        }
        prevToggle = g.b();
        if (g.y() && !prevReset) {
            headingOffset = rawHeading(feedback);
        }
        prevReset = g.y();

        double forward = deadband(-g.ly());
        double left = deadband(-g.lx());
        double omega = deadband(-g.rx()); // CCW, robot frame in both modes

        double vx = forward;
        double vy = left;
        if (fieldOriented) {
            // Stick is a field intent; rotate it by -heading into the robot frame.
            double h = rawHeading(feedback) - headingOffset;
            double cos = Math.cos(h);
            double sin = Math.sin(h);
            vx = forward * cos + left * sin;
            vy = -forward * sin + left * cos;
        }
        return Intent.of(new Drive.Manual(vx, vy, omega));
    }

    /** In C1 the localizer is Pinpoint; without a pose, heading is unknown. */
    private static double rawHeading(Feedback feedback) {
        if (feedback == null || feedback.world() == null) {
            return 0.0;
        }
        Pose pose = feedback.world().pose();
        return pose == null ? feedback.world().yaw() : pose.heading();
    }

    private static double deadband(double value) {
        if (Math.abs(value) < DEADBAND) {
            return 0.0;
        }
        // Rescale after the deadband to avoid a jump at the threshold.
        double sign = Math.signum(value);
        return sign * (Math.abs(value) - DEADBAND) / (1.0 - DEADBAND);
    }

}
