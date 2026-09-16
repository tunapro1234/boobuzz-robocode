package boobuzz.core.subsystem.pedro;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.hal.Mechanism;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Converts Pedro drivetrain output into HAL {@link RobotAction} data.
 *
 * <p>This class does not write hardware. {@link #drive(DrivePowers, boolean)} only
 * stores the latest FL/FR/BL/BR powers; RobotEngine sends {@link #lastAction()}
 * to HAL in the same tick. Motor names come from {@link Mechanism} positions.
 */
public final class HalDrivetrain implements Drivetrain {

    private static final int FL = 0;
    private static final int FR = 1;
    private static final int BL = 2;
    private static final int BR = 3;

    private final String[] motorNames;
    private final double[] wheelPowers = new double[4];
    private final boolean[] nonFiniteWarnings = new boolean[4];

    HalDrivetrain(String[] motorNames) {
        this.motorNames = motorNames.clone();
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        double[] mixed = mecanum(powers);
        for (int i = 0; i < wheelPowers.length; i++) {
            wheelPowers[i] = safePower(mixed[i], i);
        }
    }

    /**
     * Returns the largest [0,1] factor by which a power delta added in Pedro's
     * priority order can be applied without saturating a wheel.
     */
    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        double[] base = mecanum(current);
        double[] change = mecanum(delta);
        for (int i = 0; i < base.length; i++) {
            if (!Double.isFinite(base[i]) || !Double.isFinite(change[i])) {
                warnNonFinite(i, !Double.isFinite(base[i]) ? base[i] : change[i]);
                return 0.0;
            }
            if (Math.abs(base[i]) > 1.0) {
                return 0.0;
            }
        }
        double scale = 1.0;
        for (int i = 0; i < wheelPowers.length; i++) {
            if (Math.abs(change[i]) < 1e-9) {
                continue;
            }
            double positive = (1.0 - base[i]) / change[i];
            double negative = (-1.0 - base[i]) / change[i];
            if (positive >= 0.0 && positive < scale) {
                scale = positive;
            }
            if (negative >= 0.0 && negative < scale) {
                scale = negative;
            }
        }
        return RobotAction.clamp(scale, 0.0, 1.0);
    }

    @Override
    public void stop() {
        for (int i = 0; i < wheelPowers.length; i++) {
            wheelPowers[i] = 0.0;
        }
    }

    @Override
    public void stop(boolean brake) {
        stop();
    }

    @Override
    public Map<String, Object> debug() {
        return Map.of();
    }

    @Override
    public double interpolateVelocity(double forwardVelocity, double strafeVelocity,
                                      double theta) {
        if (!Double.isFinite(forwardVelocity) || !Double.isFinite(strafeVelocity)
                || !Double.isFinite(theta) || forwardVelocity < 0.0 || strafeVelocity < 0.0) {
            System.err.printf("HalDrivetrain: non-finite velocity interpolation; returning zero%n");
            return 0.0;
        }
        double forwardTerm = Math.abs(Math.cos(theta));
        double strafeTerm = Math.abs(Math.sin(theta));
        double denominator = 0.0;
        if (forwardTerm > 1e-12) {
            if (forwardVelocity <= 0.0) {
                return 0.0;
            }
            denominator += forwardTerm / forwardVelocity;
        }
        if (strafeTerm > 1e-12) {
            if (strafeVelocity <= 0.0) {
                return 0.0;
            }
            denominator += strafeTerm / strafeVelocity;
        }
        if (!Double.isFinite(denominator) || denominator <= 0.0) {
            System.err.printf("HalDrivetrain: invalid velocity interpolation; returning zero%n");
            return 0.0;
        }
        double result = 1.0 / denominator;
        if (!Double.isFinite(result)) {
            System.err.printf("HalDrivetrain: non-finite velocity interpolation; returning zero%n");
            return 0.0;
        }
        return result;
    }

    /** Final motor command to write to HAL for this tick. */
    public RobotAction lastAction() {
        Map<String, Double> motors = new LinkedHashMap<>(4);
        motors.put(motorNames[FL], safePower(wheelPowers[FL], FL));
        motors.put(motorNames[FR], safePower(wheelPowers[FR], FR));
        motors.put(motorNames[BL], safePower(wheelPowers[BL], BL));
        motors.put(motorNames[BR], safePower(wheelPowers[BR], BR));
        return RobotAction.ofMotors(motors);
    }

    static double[] normalizedMecanum(DrivePowers powers) {
        double[] wheelPowers = mecanum(powers);
        double peak = 1.0;
        for (double power : wheelPowers) {
            if (!Double.isFinite(power)) {
                System.err.printf("HalDrivetrain: non-finite manual mix; zeroing all wheel powers%n");
                return new double[] {0.0, 0.0, 0.0, 0.0};
            }
            peak = Math.max(peak, Math.abs(power));
        }
        for (int i = 0; i < wheelPowers.length; i++) {
            double normalized = wheelPowers[i] / peak;
            wheelPowers[i] = Double.isFinite(normalized)
                    ? RobotAction.clamp(normalized, -1.0, 1.0) : 0.0;
        }
        return wheelPowers;
    }

    private double safePower(double value, int wheel) {
        if (!Double.isFinite(value)) {
            warnNonFinite(wheel, value);
            return 0.0;
        }
        return RobotAction.clamp(value, -1.0, 1.0);
    }

    private void warnNonFinite(int wheel, double value) {
        if (!nonFiniteWarnings[wheel]) {
            nonFiniteWarnings[wheel] = true;
            System.err.printf("HalDrivetrain: non-finite %s power (%s); zeroing%n",
                    cornerName(wheel), value);
        }
    }

    private static String cornerName(int wheel) {
        return switch (wheel) {
            case FL -> "fl";
            case FR -> "fr";
            case BL -> "bl";
            case BR -> "br";
            default -> "wheel" + wheel;
        };
    }

    private static double[] mecanum(DrivePowers powers) {
        double forward = powers.forward();
        double strafe = powers.strafe();
        double turn = powers.turn();
        return new double[] {
                forward - strafe - turn,
                forward + strafe + turn,
                forward + strafe - turn,
                forward - strafe + turn};
    }
}
