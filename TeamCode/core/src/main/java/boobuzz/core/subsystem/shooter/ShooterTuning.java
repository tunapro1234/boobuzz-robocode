package boobuzz.core.subsystem.shooter;

import boobuzz.core.hal.RobotConstants;

/**
 * One immutable snapshot of the live-tunable shooter values (archive
 * ShooterPidfPowerStorage). The shooter reads exactly one snapshot per tick, so gains
 * never mix within a tick.
 */
public record ShooterTuning(double kS, double kV, double kP, double kI, double kD,
                            double integralZoneRpm, double integralMaxAccum,
                            double toleranceRpm, long stabilityMs) {

    public static final ShooterTuning DEFAULTS = new ShooterTuning(
            RobotConstants.SHOOTER_KS, RobotConstants.SHOOTER_KV, RobotConstants.SHOOTER_KP,
            RobotConstants.SHOOTER_KI, RobotConstants.SHOOTER_KD,
            RobotConstants.SHOOTER_INTEGRAL_ZONE_RPM, RobotConstants.SHOOTER_INTEGRAL_MAX_ACCUM,
            RobotConstants.SHOOTER_TOLERANCE_RPM, RobotConstants.SHOOTER_STABILITY_MS);

    /** Gains must be finite; zone, clamp, tolerance and dwell must also be non-negative. */
    public boolean isValid() {
        return Double.isFinite(kS) && Double.isFinite(kV) && Double.isFinite(kP)
                && Double.isFinite(kI) && Double.isFinite(kD)
                && nonNegative(integralZoneRpm) && nonNegative(integralMaxAccum)
                && nonNegative(toleranceRpm) && stabilityMs >= 0;
    }

    private static boolean nonNegative(double value) {
        return Double.isFinite(value) && value >= 0.0;
    }
}
