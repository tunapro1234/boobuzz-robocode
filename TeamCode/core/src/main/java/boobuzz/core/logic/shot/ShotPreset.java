package boobuzz.core.logic.shot;

import boobuzz.core.hal.RobotConstants;

/**
 * Fixed shot setpoint: flywheel RPM, hood angle and robot-relative turret angle.
 * Shared by every real profile; not a calibrated distance solution.
 */
public record ShotPreset(double rpm, double hoodDeg, double turretRad) {

    /** Archive recovery preset (RPM 4000, hood 45 deg, turret 0 deg). */
    public static final ShotPreset DEFAULT = new ShotPreset(
            RobotConstants.SHOT_PRESET_RPM,
            RobotConstants.SHOT_PRESET_HOOD_DEG,
            Math.toRadians(RobotConstants.SHOT_PRESET_TURRET_DEG));

    public ShotPreset {
        String problem = validate(rpm, hoodDeg, turretRad);
        if (problem != null) {
            throw new IllegalArgumentException(problem);
        }
    }

    /** Returns null when valid, otherwise a short reason; out-of-range values are never clamped. */
    public static String validate(double rpm, double hoodDeg, double turretRad) {
        if (!Double.isFinite(rpm) || rpm <= 0.0) {
            return "preset rpm must be positive and finite";
        }
        if (!Double.isFinite(hoodDeg)
                || hoodDeg < RobotConstants.HOOD_MIN_DEG || hoodDeg > RobotConstants.HOOD_MAX_DEG) {
            return "preset hood outside " + RobotConstants.HOOD_MIN_DEG
                    + ".." + RobotConstants.HOOD_MAX_DEG + " deg";
        }
        double turretDeg = Math.toDegrees(turretRad);
        if (!Double.isFinite(turretRad)
                || turretDeg < RobotConstants.TURRET_MIN_DEG || turretDeg > RobotConstants.TURRET_MAX_DEG) {
            return "preset turret outside " + RobotConstants.TURRET_MIN_DEG
                    + ".." + RobotConstants.TURRET_MAX_DEG + " deg";
        }
        return null;
    }
}
