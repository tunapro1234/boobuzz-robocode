package org.firstinspires.ftc.teamcode.tuning;

import boobuzz.core.subsystem.shooter.ShooterTuning;

import com.acmerobotics.dashboard.config.Config;

/**
 * Dashboard-tunable shooter values, like the archive ShooterPidfPowerStorage. Boot
 * values come from RobotConstants; the shooter reads one snapshot per tick.
 */
@Config
public final class ShooterTuningConfig {

    public static double kS = ShooterTuning.DEFAULTS.kS();
    public static double kV = ShooterTuning.DEFAULTS.kV();
    public static double kP = ShooterTuning.DEFAULTS.kP();
    public static double kI = ShooterTuning.DEFAULTS.kI();
    public static double kD = ShooterTuning.DEFAULTS.kD();
    public static double integralZoneRpm = ShooterTuning.DEFAULTS.integralZoneRpm();
    public static double integralMaxAccum = ShooterTuning.DEFAULTS.integralMaxAccum();
    public static double targetToleranceRpm = ShooterTuning.DEFAULTS.toleranceRpm();
    public static long stabilityDurationMs = ShooterTuning.DEFAULTS.stabilityMs();

    private ShooterTuningConfig() {}

    /** Current dashboard values as one snapshot; the shooter rejects invalid ones. */
    public static ShooterTuning snapshot() {
        return new ShooterTuning(kS, kV, kP, kI, kD, integralZoneRpm, integralMaxAccum,
                targetToleranceRpm, stabilityDurationMs);
    }
}
