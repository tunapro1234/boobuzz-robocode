package boobuzz.core.logic.cplx_engine_1;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;

/** Pedro follower ayarlarinin tek kaynagi. */
public final class PedroConstants {

    private PedroConstants() {}

    // Gecen sezonun robotundan baslangic degerleri; YENI ROBOTTA YENIDEN OLCULECEK.
    public static final double FORWARD_ZERO_POWER_ACCELERATION = -36.17;
    public static final double LATERAL_ZERO_POWER_ACCELERATION = -85.98;
    public static final double MAX_FORWARD_VELOCITY = 73.63;
    public static final double MAX_STRAFE_VELOCITY = 54.09;

    // Pedro 3.0 icin muhafazakar baslangic kontrolculeri; AutoTune ile yenilenecek.
    public static final double HEADING_KP = 1.0;
    public static final double FORWARD_TRANSLATIONAL_KP = 0.10;
    public static final double STRAFE_TRANSLATIONAL_KP = 0.10;
    public static final double VELOCITY_FEEDFORWARD = 1.0 / MAX_FORWARD_VELOCITY;

    /** Her follower icin durum tasimayan ayarlardan yeni bir config uretir. */
    public static ForesightConfig createForesightConfig() {
        return new ForesightConfig(c -> {
            c.headingFeedback.set(Controller.proportional(HEADING_KP));
            c.forwardTranslational.set(Controller.proportional(FORWARD_TRANSLATIONAL_KP));
            c.strafeTranslational.set(Controller.proportional(STRAFE_TRANSLATIONAL_KP));
            c.coast.set(Controller.proportionalFeedforward(VELOCITY_FEEDFORWARD));
            c.brake.set(Controller.proportionalFeedforward(VELOCITY_FEEDFORWARD));

            // Gecen sezonun sifir-guc yavaslamasindan turetilen ilk yaklasim.
            // AutoTune lineer/kuadratik fren katsayilarinin asil kaynagidir.
            c.linearBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
            c.quadraticBrakeCoefficients.set(Matrix.diag(
                    1.0 / (2.0 * Math.abs(FORWARD_ZERO_POWER_ACCELERATION)),
                    1.0 / (2.0 * Math.abs(LATERAL_ZERO_POWER_ACCELERATION))));
            c.headingBrakeCoefficients.set(Vector2D.zero());

            c.maxAchievableForwardVelocity.set(MAX_FORWARD_VELOCITY);
            c.maxAchievableStrafeVelocity.set(MAX_STRAFE_VELOCITY);
            c.naturalForwardDeceleration.set(Math.abs(FORWARD_ZERO_POWER_ACCELERATION));
            c.naturalStrafeDeceleration.set(Math.abs(LATERAL_ZERO_POWER_ACCELERATION));
        });
    }

    public static Follower createFollower(HalLocalizer localizer, HalDrivetrain drivetrain) {
        return new Follower(localizer, drivetrain,
                new Foresight(createForesightConfig()));
    }
}
