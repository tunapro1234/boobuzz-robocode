package boobuzz.core.logic.cplx_engine_1;

import boobuzz.core.hal.Mechanism;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;

/** Single source of Pedro follower settings. */
public final class PedroConstants {

    private PedroConstants() {}

    // Conservative starting controllers for Pedro 3.0; tune with AutoTune.
    private static final double HEADING_KP = 1.0;
    private static final double FORWARD_TRANSLATIONAL_KP = 0.10;
    private static final double STRAFE_TRANSLATIONAL_KP = 0.10;

    /** Creates a new config from stateless settings for each follower. */
    public static ForesightConfig createForesightConfig(Mechanism mechanism) {
        Mechanism.Physics physics = mechanism.physics();
        double maxForwardVelocity = maxForwardVelocity(mechanism);
        double maxStrafeVelocity = maxForwardVelocity * physics.strafeEfficiency();
        double forwardDeceleration = physics.zeroPowerDecelForwardInchesPerSecondSquared();
        double lateralDeceleration = physics.zeroPowerDecelLateralInchesPerSecondSquared();
        requirePositive("physics.strafe_eff", physics.strafeEfficiency());
        requirePositive("physics.zero_power_decel_forward_in_s2", forwardDeceleration);
        requirePositive("physics.zero_power_decel_lateral_in_s2", lateralDeceleration);
        double velocityFeedforward = 1.0 / maxForwardVelocity;

        return new ForesightConfig(c -> {
            c.headingFeedback.set(Controller.proportional(HEADING_KP));
            c.forwardTranslational.set(Controller.proportional(FORWARD_TRANSLATIONAL_KP));
            c.strafeTranslational.set(Controller.proportional(STRAFE_TRANSLATIONAL_KP));
            c.coast.set(Controller.proportionalFeedforward(velocityFeedforward));
            c.brake.set(Controller.proportionalFeedforward(velocityFeedforward));

            // Initial approximation derived from last season's zero-power deceleration.
            // This is the source for AutoTune's linear/quadratic brake coefficients.
            c.linearBrakeCoefficients.set(Matrix.diag(0.0, 0.0));
            c.quadraticBrakeCoefficients.set(Matrix.diag(
                    1.0 / (2.0 * forwardDeceleration),
                    1.0 / (2.0 * lateralDeceleration)));
            c.headingBrakeCoefficients.set(Vector2D.zero());

            c.maxAchievableForwardVelocity.set(maxForwardVelocity);
            c.maxAchievableStrafeVelocity.set(maxStrafeVelocity);
            c.naturalForwardDeceleration.set(forwardDeceleration);
            c.naturalStrafeDeceleration.set(lateralDeceleration);
        });
    }

    public static Follower createFollower(Mechanism mechanism, HalLocalizer localizer,
                                          HalDrivetrain drivetrain) {
        return new Follower(localizer, drivetrain,
                new Foresight(createForesightConfig(mechanism)));
    }

    private static double maxForwardVelocity(Mechanism mechanism) {
        double wheelCircumference = Math.PI * mechanism.drivetrain().wheelDiameter();
        requirePositive("drivetrain.wheel_diameter", mechanism.drivetrain().wheelDiameter());
        double velocity = Double.POSITIVE_INFINITY;
        for (String name : mechanism.wheelMotorNames()) {
            Mechanism.Motor motor = mechanism.motor(name);
            double efficiency = mechanism.physics().efficiency().getOrDefault(name, 1.0);
            requirePositive("motors." + name + ".free_rpm", motor.freeRpm());
            requirePositive("physics.efficiency." + name, efficiency);
            velocity = Math.min(velocity,
                    motor.freeRpm() * efficiency * wheelCircumference / 60.0);
        }
        if (!Double.isFinite(velocity)) {
            throw new Mechanism.MechanismException("no wheel motor found for Pedro");
        }
        return velocity;
    }

    private static void requirePositive(String field, double value) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new Mechanism.MechanismException(
                    "mechanism.yaml '" + field + "' must be positive: " + value);
        }
    }
}
