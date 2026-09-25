package boobuzz.core.hal;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Arrays;
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
    // Provisional B02 pollen-intake fixture geometry, in inches.
    public static final double INTAKE_MOUTH_FORWARD_IN = 9.0;
    public static final double INTAKE_OPENING_IN = 3.2;
    public static final double INTAKE_CAPTURE_DEPTH_IN = 2.0;
    public static final double INTAKE_CAPACITY = 3.0;
    public static final double POLLEN_DIAMETER_IN = 2.8;
    public static final double NECTAR_DIAMETER_IN = 3.6;
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

    // Archive FeederPowerSubsystem: HardwareConstants.FeederPower.feedPower/singleBallDurationMs.
    public static final double FEEDER_PULSE_POWER = 1.0;
    public static final long FEEDER_PULSE_MS = 350;
    // Archive lvbelc5 ShootingController.FEEDER_DELAY_MS (post-pulse delay, not pulse length).
    public static final long FEEDER_POST_PULSE_DELAY_MS = 100;

    // Archive HardwareConstants.ShooterPIDF (NiMh 09-12-25): boot defaults of the
    // dashboard-tunable ShooterPidfPowerStorage. Gains are power per RPM units.
    public static final double SHOOTER_KS = 0.18766200;
    public static final double SHOOTER_KV = 0.00013514;
    public static final double SHOOTER_KP = 0.00030984;
    public static final double SHOOTER_KI = 0.00189683;
    public static final double SHOOTER_KD = 1.26816e-05;
    public static final double SHOOTER_INTEGRAL_ZONE_RPM = 250.0;
    public static final double SHOOTER_INTEGRAL_MAX_ACCUM = 3000.0;
    public static final double SHOOTER_TOLERANCE_RPM = 100.0;
    public static final long SHOOTER_STABILITY_MS = 150;
    // Archive ShooterPidfPowerSubsystem literals: FF off below 1500 RPM, stability reset on
    // a >50 RPM target change, 1 ms minimum dt.
    public static final double SHOOTER_FF_MIN_RPM = 1500.0;
    public static final double SHOOTER_TARGET_RESET_RPM = 50.0;
    public static final double SHOOTER_MIN_DT_S = 1e-3;
    // Archive HardwareConstants.Shooter: left = right * followerScale; wheel = motor * 1.6.
    public static final double SHOOTER_FOLLOWER_SCALE = 1.0;
    public static final double SHOOTER_MOTOR_TO_WHEEL = 1.6;

    // Timing-only defaults for the phase-1.1 simulator mechanism stubs.
    public static final double STUB_SPINUP_S = 0.5;
    public static final double STUB_FEED_S = 0.2;
    public static final double STUB_TURRET_SETTLE_S = 0.3;

    public static final boolean ALLIANCE_BLUE = true;
    public static final double GOAL_X = 48.0;
    public static final double RED_GOAL_X = 96.0;
    public static final double GOAL_Y = 96.0;
    public static final double SHOOTER_RPM_BASE = 300.0;
    public static final double SHOOTER_RPM_PER_IN = 1.0;
    public static final double SHOOTER_HOOD_BASE = 0.0;
    public static final double SHOOTER_HOOD_PER_IN = 0.0;

    // Archive HardwareConstants.Hood: 25..50 deg hood maps to 0..215 deg of 300-deg servo travel.
    public static final double HOOD_MIN_DEG = 25.0;
    public static final double HOOD_MAX_DEG = 50.0;
    public static final double HOOD_SERVO_TRAVEL_DEG = 215.0;
    public static final double HOOD_SERVO_RANGE_DEG = 300.0;
    public static final double HOOD_DEFAULT_DEG = 44.0;
    public static final double HOOD_STOW_DEG = 25.0;
    // Archive lvbelc5 RecoveryController.DEFAULT_HOOD_ANGLE.
    public static final double HOOD_RECOVERY_DEG = 45.0;
    // B06 simulator fixture, not hardware data: hood rate and settling margin.
    public static final double HOOD_FIXTURE_RATE_DEG_S = 90.0;
    public static final long HOOD_SETTLE_MARGIN_MS = 100;

    // Archive HardwareConstants.Turret: range, gearing (encoder 0.715 rev per turret rev),
    // shared shooterLeft encoder reversed, max power.
    public static final double TURRET_MIN_DEG = -90.0;
    public static final double TURRET_MAX_DEG = 90.0;
    public static final double TURRET_ENCODER_TICKS_PER_REV = 8192.0;
    public static final double TURRET_ENCODER_GEAR_TEETH = 1.0;
    public static final double TURRET_GEAR_TEETH = 0.715;
    public static final boolean TURRET_ENCODER_REVERSED = true;
    public static final double TURRET_MAX_POWER = 1.0;
    // Archive HardwareConstants.TurretPidPazar PID (kS 0); FTCLib 2.1.1 totalError bound 1.
    public static final double TURRET_KP = 0.0171;
    public static final double TURRET_KI = 0.0401;
    public static final double TURRET_KD = 0.002;
    public static final double TURRET_INTEGRAL_MAX = 1.0;
    // Archive TurretPidPazar absolute analog: 0..3.3 V over one shaft turn, shaft 125 deg = 0.
    public static final double TURRET_ANALOG_MIN_V = 0.0;
    public static final double TURRET_ANALOG_MAX_V = 3.3;
    public static final double TURRET_ANALOG_SHAFT_OFFSET_DEG = 125.0;
    // Archive TurretPidPazar estimator and startup calibration window.
    public static final double TURRET_KALMAN_Q = 0.1;
    public static final double TURRET_KALMAN_R = 50.0;
    public static final double TURRET_ANALOG_LPF_ALPHA = 0.1;
    public static final double TURRET_FULL_TRUST_S = 0.75;
    public static final double TURRET_FADE_OUT_S = 1.25;
    // Archive snapped estimates within 15 deg of 0 to 0; spec B07 disables it in v0 so a
    // real offset is never concealed.
    public static final boolean TURRET_SNAP_TO_ZERO_ENABLED = false;
    public static final double TURRET_SNAP_TO_ZERO_THRESHOLD_DEG = 15.0;
    // Spec B07 fixture values (not archive): soft margin inside each hard stop, and the
    // settled criterion (error <= 2 deg and rate <= 5 deg/s for 100 ms).
    public static final double TURRET_SOFT_MARGIN_DEG = 5.0;
    public static final double TURRET_SETTLE_TOL_DEG = 2.0;
    public static final double TURRET_SETTLE_RATE_DEG_S = 5.0;
    public static final long TURRET_SETTLE_MS = 100;

    // TeleOp BACK reset pose: the fixed blue-side field pose used last season.
    public static final double TELEOP_RESET_POSE_X = 24.0;
    public static final double TELEOP_RESET_POSE_Y = 96.0;
    public static final double TELEOP_RESET_POSE_H = 1.5707963267948966;

    // Simulator default; real robot entry points use REAL_DEBUG_TAP_PORT for safety.
    public static final int DEBUG_TAP_PORT = 5600;
    public static final int REAL_DEBUG_TAP_PORT = 0;
    public static final int CONTROL_SOCKET_PORT = 5601;
    public static final int CONTROL_SOCKET_TIMEOUT_MS = 250;
    public static final int SIM_CONNECT_TIMEOUT_MS = 5000;
    public static final int SIM_READ_TIMEOUT_MS = 5000;
    public static final String DEFAULT_CONTROLLER = "gamepad";

    // Protocol wheel keys stay stable; FTC HardwareMap names are declared below.
    // free_rpm = 73.63 in/s * 60 / (pi * 4 in).
    public static final Motor FL = new Motor("fl", "wheel", 6.5, 5.5, 45.0, 537.7, 351.55735379568756);
    public static final Motor FR = new Motor("fr", "wheel", 6.5, -5.5, -45.0, 537.7, 351.55735379568756);
    public static final Motor BL = new Motor("bl", "wheel", -6.5, 5.5, -45.0, 537.7, 351.55735379568756);
    public static final Motor BR = new Motor("br", "wheel", -6.5, -5.5, 45.0, 537.7, 351.55735379568756);
    public static final Motor[] MOTORS = {FL, FR, BL, BR};

    public static final String SHOOTER_RIGHT_MOTOR_NAME = "shooterRight";
    public static final String SHOOTER_LEFT_MOTOR_NAME = "shooterLeft";
    public static final String SHOOTER_FEEDBACK_ENCODER_NAME = "shooterRight";
    public static final String INTAKE_MOTOR_NAME = "intake";
    public static final String FEEDER_MOTOR_NAME = "feeder";
    public static final String TURRET_PRIMARY_SERVO_NAME = "turret_servo";
    public static final String TURRET_SECONDARY_SERVO_NAME = "turret_servo2";
    public static final String TURRET_ENCODER_NAME = "shooterLeft";
    public static final String TURRET_ANALOG_NAME = "turret_analog";
    public static final String HOOD_LEFT_SERVO_NAME = "hood_left";
    public static final String HOOD_RIGHT_SERVO_NAME = "hood_right";
    public static final String INTAKE_DISTANCE_NAME = "intake_dist";
    public static final String LIMELIGHT_NAME = "limelight";
    public static final String LEFT_FRONT_MOTOR_NAME = "leftFront";
    public static final String RIGHT_FRONT_MOTOR_NAME = "rightFront";
    public static final String LEFT_BACK_MOTOR_NAME = "leftBack";
    public static final String RIGHT_BACK_MOTOR_NAME = "rightBack";

    public static final DcDevice INTAKE = new DcDevice(INTAKE_MOTOR_NAME, "REVERSE", "BRAKE", 28.0, 6000.0);
    public static final DcDevice FEEDER = new DcDevice(FEEDER_MOTOR_NAME, "FORWARD", "BRAKE", 28.0, 6000.0);
    public static final DcDevice SHOOTER_RIGHT = new DcDevice(SHOOTER_RIGHT_MOTOR_NAME, "REVERSE", "FLOAT", 28.0, 6000.0);
    public static final DcDevice SHOOTER_LEFT = new DcDevice(SHOOTER_LEFT_MOTOR_NAME, "FORWARD", "FLOAT", 28.0, 6000.0);
    public static final DcDevice[] DC_DEVICES = {INTAKE, FEEDER, SHOOTER_RIGHT, SHOOTER_LEFT};
    public static final CrServo TURRET_PRIMARY = new CrServo(TURRET_PRIMARY_SERVO_NAME, "FORWARD");
    public static final CrServo TURRET_SECONDARY = new CrServo(TURRET_SECONDARY_SERVO_NAME, "FORWARD");
    public static final CrServo[] CR_SERVOS = {TURRET_PRIMARY, TURRET_SECONDARY};
    public static final String[] ANALOG_INPUTS = {TURRET_ANALOG_NAME};
    // Archive HoodSubsystem never set a servo direction: the left inversion is already the
    // complementary 1-u command (rightInverse=false). A HAL REVERSE here would invert twice.
    public static final PosServo HOOD_LEFT = new PosServo(HOOD_LEFT_SERVO_NAME, "FORWARD", 1.0);
    public static final PosServo HOOD_RIGHT = new PosServo(HOOD_RIGHT_SERVO_NAME, "FORWARD", 0.0);
    public static final PosServo[] SERVOS = {HOOD_LEFT, HOOD_RIGHT};
    public static final String[] ENCODERS = {"leftFront", "rightFront", "leftBack", "rightBack", "intake", "feeder", "shooterRight", "shooterLeft"};

    // MEASURED Pedro 2.x Constants.java values from last season; re-measure on the new chassis.
    public static final Pinpoint PINPOINT = new Pinpoint(161.0, 0.0, "FORWARD", "REVERSED", "goBILDA_4_BAR_POD");

    /** Returns the one shared in-memory mechanism built from these constants. */
    public static Mechanism mechanism() {
        return Mechanism.DEFAULT;
    }

    /** Stable hash for bag headers; it is derived from all machine-read constants. */
    public static String constantsHash() {
        return constantsHashForMass(ROBOT_MASS_KG);
    }

    // Package-private seam for the mass-sensitivity regression test; production uses ROBOT_MASS_KG above.
    static String constantsHashForMass(double robotMassKg) {
        StringBuilder source = new StringBuilder()
                .append(ROBOT_WIDTH).append('|').append(ROBOT_LENGTH).append('|')
                .append(robotMassKg).append('|')
                .append(INTAKE_MOUTH_FORWARD_IN).append('|')
                .append(INTAKE_OPENING_IN).append('|')
                .append(INTAKE_CAPTURE_DEPTH_IN).append('|')
                .append(INTAKE_CAPACITY).append('|')
                .append(POLLEN_DIAMETER_IN).append('|')
                .append(NECTAR_DIAMETER_IN).append('|')
                .append(WHEEL_DIAMETER).append('|').append(BATTERY_V).append('|')
                .append(MOTOR_TAU_S).append('|')
                .append(EFFICIENCY_FL).append('|').append(EFFICIENCY_FR).append('|')
                .append(EFFICIENCY_BL).append('|').append(EFFICIENCY_BR).append('|')
                .append(STRAFE_EFF).append('|')
                .append(ZERO_POWER_DECEL_FORWARD_IN_S2).append('|')
                .append(ZERO_POWER_DECEL_LATERAL_IN_S2).append('|');
        for (Motor motor : MOTORS) {
            source.append(motor).append('|');
        }
        for (DcDevice device : DC_DEVICES) {
            source.append(device).append('|');
        }
        for (CrServo servo : CR_SERVOS) {
            source.append(servo).append('|');
        }
        for (PosServo servo : SERVOS) {
            source.append(servo).append('|');
        }
        source.append(Arrays.toString(ENCODERS)).append('|');
        source.append(PINPOINT).append('|').append(TELEOP_RESET_POSE_X).append('|')
                .append(TELEOP_RESET_POSE_Y).append('|').append(TELEOP_RESET_POSE_H);
        try {
            byte[] digest = java.security.MessageDigest.getInstance("SHA-256")
                    .digest(source.toString().getBytes(java.nio.charset.StandardCharsets.UTF_8));
            StringBuilder hex = new StringBuilder(digest.length * 2);
            for (byte value : digest) hex.append(String.format(java.util.Locale.ROOT, "%02x", value));
            return hex.toString();
        } catch (java.security.NoSuchAlgorithmException e) {
            throw new IllegalStateException("SHA-256 is unavailable", e);
        }
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
        List<String> motorNames = new java.util.ArrayList<>();
        for (Motor motor : MOTORS) motorNames.add(motor.name());
        for (DcDevice device : DC_DEVICES) motorNames.add(device.name());
        for (CrServo servo : CR_SERVOS) motorNames.add(servo.name());
        List<String> servoNames = new java.util.ArrayList<>();
        for (PosServo servo : SERVOS) servoNames.add(servo.name());
        return new Mechanism(
                motorNames,
                servoNames,
                motors,
                new Mechanism.Drivetrain(WHEEL_DIAMETER),
                new Mechanism.Pinpoint(
                        PINPOINT.xPodOffsetMm(), PINPOINT.yPodOffsetMm(),
                        PINPOINT.xPodDirection(), PINPOINT.yPodDirection(), PINPOINT.podType()),
                new Mechanism.Physics(
                        efficiency,
                        STRAFE_EFF,
                        ZERO_POWER_DECEL_FORWARD_IN_S2,
                        ZERO_POWER_DECEL_LATERAL_IN_S2),
                Arrays.asList(DC_DEVICES),
                Arrays.asList(CR_SERVOS),
                Arrays.asList(SERVOS),
                Arrays.asList(ENCODERS));
    }

    /** Machine-readable motor configuration in protocol order. */
    public record Motor(String name, String drives, double xForward, double yLeft,
                        double rollerDeg, double ticksPerRev, double freeRpm) {}

    public record DcDevice(String name, String direction, String zeroPower,
                           double ticksPerRev, double freeRpm) {}

    public record CrServo(String name, String direction) {}

    public record PosServo(String name, String direction, double initialPos) {}

    /** Machine-readable Pinpoint configuration. */
    public record Pinpoint(double xPodOffsetMm, double yPodOffsetMm,
                           String xPodDirection, String yPodDirection, String podType) {}
}
