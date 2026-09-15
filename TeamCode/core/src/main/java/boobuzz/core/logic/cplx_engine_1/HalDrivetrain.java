package boobuzz.core.logic.cplx_engine_1;

import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.Mechanism;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Pedro drivetrain cikisini HAL {@link RobotAction} verisine cevirir.
 *
 * <p>Bu sinif donanima yazmaz. {@link #drive(DrivePowers, boolean)} yalnizca son
 * FL/FR/BL/BR guclerini saklar; RobotEngine ayni tick'te {@link #lastAction()}
 * sonucunu HAL'e yollar. Motor adlari {@link Mechanism} konumlarindan gelir.
 */
public final class HalDrivetrain implements Drivetrain {

    private static final int FL = 0;
    private static final int FR = 1;
    private static final int BL = 2;
    private static final int BR = 3;

    private final String[] motorNames;
    private final double[] wheelPowers = new double[4];

    HalDrivetrain(String[] motorNames) {
        this.motorNames = motorNames.clone();
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        double[] mixed = mecanum(powers);
        for (int i = 0; i < wheelPowers.length; i++) {
            wheelPowers[i] = RobotAction.clamp(mixed[i], -1.0, 1.0);
        }
    }

    /**
     * Pedro'nun oncelik sirasiyla ekledigi bir guc deltasinin tekerlekleri
     * doyurmadan uygulanabilecek en buyuk [0,1] katsayisini verir.
     */
    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        double[] base = mecanum(current);
        double[] change = mecanum(delta);
        for (double power : base) {
            if (Math.abs(power) > 1.0) {
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
        return 1.0 / (Math.abs(Math.cos(theta)) / forwardVelocity
                + Math.abs(Math.sin(theta)) / strafeVelocity);
    }

    /** O tick'te HAL'e yazilacak son motor komutu. */
    public RobotAction lastAction() {
        Map<String, Double> motors = new LinkedHashMap<>(4);
        motors.put(motorNames[FL], wheelPowers[FL]);
        motors.put(motorNames[FR], wheelPowers[FR]);
        motors.put(motorNames[BL], wheelPowers[BL]);
        motors.put(motorNames[BR], wheelPowers[BR]);
        return RobotAction.ofMotors(motors);
    }

    static double[] normalizedMecanum(DrivePowers powers) {
        double[] wheelPowers = mecanum(powers);
        double peak = 1.0;
        for (double power : wheelPowers) {
            peak = Math.max(peak, Math.abs(power));
        }
        for (int i = 0; i < wheelPowers.length; i++) {
            wheelPowers[i] = RobotAction.clamp(wheelPowers[i] / peak, -1.0, 1.0);
        }
        return wheelPowers;
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
