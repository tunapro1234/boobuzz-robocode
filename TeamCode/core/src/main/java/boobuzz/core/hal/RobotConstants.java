package boobuzz.core.hal;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Compile-time robot configuration shared by the real robot and simulator.
 * Values are intentionally plain constants so configuration errors fail at compile time.
 */
public final class RobotConstants {

    private RobotConstants() {}

    public static final double ROBOT_WIDTH = 18.0;
    public static final double ROBOT_LENGTH = 18.0;
    public static final double ROBOT_MASS_KG = 12.0;
    public static final String ANGLE_UNIT = "deg";
    public static final String DRIVETRAIN_TYPE = "mecanum";
    public static final double WHEEL_DIAMETER = 4.0;

    public static final double BATTERY_V = 12.0;
    public static final double MOTOR_TAU_S = 0.1;
    public static final double EFFICIENCY_FL = 1.0;
    public static final double EFFICIENCY_FR = 1.0;
    public static final double EFFICIENCY_BL = 1.0;
    public static final double EFFICIENCY_BR = 1.0;

    // Calibration source: archive/ftc/de-cock TeamCode pedroPathing/Constants.java (last season).
    // yVelocity/xVelocity = 54.09/73.63.
    public static final double STRAFE_EFF = 0.7346;
    // |forwardZeroPowerAcceleration|.
    public static final double ZERO_POWER_DECEL_FORWARD_IN_S2 = 36.17;
    // |lateralZeroPowerAcceleration|.
    public static final double ZERO_POWER_DECEL_LATERAL_IN_S2 = 85.98;

    // Timing-only defaults for the phase-1.1 simulator mechanism stubs.
    public static final double STUB_SPINUP_S = 0.5;
    public static final double STUB_FEED_S = 0.2;
    public static final double STUB_TURRET_SETTLE_S = 0.3;

    // free_rpm = 73.63 in/s * 60 / (pi * 4 in); no direct RPM data was found.
    public static final Motor FL = new Motor("fl", "wheel", 6.5, 5.5, 45.0, 537.7, 351.55735379568756);
    public static final Motor FR = new Motor("fr", "wheel", 6.5, -5.5, -45.0, 537.7, 351.55735379568756);
    public static final Motor BL = new Motor("bl", "wheel", -6.5, 5.5, -45.0, 537.7, 351.55735379568756);
    public static final Motor BR = new Motor("br", "wheel", -6.5, -5.5, 45.0, 537.7, 351.55735379568756);
    public static final Motor[] MOTORS = {FL, FR, BL, BR};

    public static final String[] SERVOS = {};

    // MEASURED Pedro 2.x Constants.java values from last season; re-measure on the new chassis.
    public static final Pinpoint PINPOINT = new Pinpoint(161.0, 0.0, "FORWARD", "REVERSED", "goBILDA_4_BAR_POD");

    /** Returns the one shared in-memory mechanism built from these constants. */
    public static Mechanism mechanism() {
        return Mechanism.DEFAULT;
    }

    static Mechanism buildMechanism() {
        Map<String, Mechanism.Motor> motors = new LinkedHashMap<>();
        for (Motor motor : MOTORS) {
            motors.put(motor.name(), new Mechanism.Motor(
                    motor.drives(), motor.xForward(), motor.yLeft(), motor.freeRpm()));
        }
        Map<String, Double> efficiency = new LinkedHashMap<>();
        efficiency.put(FL.name(), EFFICIENCY_FL);
        efficiency.put(FR.name(), EFFICIENCY_FR);
        efficiency.put(BL.name(), EFFICIENCY_BL);
        efficiency.put(BR.name(), EFFICIENCY_BR);
        return new Mechanism(
                List.of(FL.name(), FR.name(), BL.name(), BR.name()),
                List.of(SERVOS),
                motors,
                new Mechanism.Drivetrain(WHEEL_DIAMETER),
                new Mechanism.Pinpoint(
                        PINPOINT.xPodOffsetMm(), PINPOINT.yPodOffsetMm(),
                        PINPOINT.xPodDirection(), PINPOINT.yPodDirection(), PINPOINT.podType()),
                new Mechanism.Physics(
                        efficiency,
                        STRAFE_EFF,
                        ZERO_POWER_DECEL_FORWARD_IN_S2,
                        ZERO_POWER_DECEL_LATERAL_IN_S2));
    }

    /** Machine-readable motor configuration in protocol order. */
    public record Motor(String name, String drives, double xForward, double yLeft,
                        double rollerDeg, double ticksPerRev, double freeRpm) {}

    /** Machine-readable Pinpoint configuration. */
    public record Pinpoint(double xPodOffsetMm, double yPodOffsetMm,
                           String xPodDirection, String yPodDirection, String podType) {}
}
