package boobuzz.core.logic.drive;

import boobuzz.core.logic.engine.C1DriveEngine;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.mechanism.Mechanism;

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
    private DrivePowers drivePowers = DrivePowers.zero();
    private boolean manual;

    public HalDrivetrain(Mechanism mechanism) {
        C1DriveEngine mapping = new C1DriveEngine(mechanism);
        motorNames = new String[] {
                mapping.frontLeftMotor(), mapping.frontRightMotor(),
                mapping.backLeftMotor(), mapping.backRightMotor()};
    }

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        drivePowers = powers;
        this.manual = manual;

        double forward = powers.forward();
        double strafe = powers.strafe();
        double turn = powers.turn();
        wheelPowers[FL] = clip(forward - strafe - turn);
        wheelPowers[FR] = clip(forward + strafe + turn);
        wheelPowers[BL] = clip(forward + strafe - turn);
        wheelPowers[BR] = clip(forward - strafe + turn);
    }

    /**
     * Pedro'nun oncelik sirasiyla ekledigi bir guc deltasinin tekerlekleri
     * doyurmadan uygulanabilecek en buyuk [0,1] katsayisini verir.
     */
    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        double[] base = unnormalized(current);
        double[] change = unnormalized(delta);
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
        return clip01(scale);
    }

    @Override
    public void stop() {
        stop(false);
    }

    @Override
    public void stop(boolean brake) {
        drivePowers = DrivePowers.zero();
        manual = false;
        for (int i = 0; i < wheelPowers.length; i++) {
            wheelPowers[i] = 0.0;
        }
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("forward", drivePowers.forward());
        out.put("strafe", drivePowers.strafe());
        out.put("turn", drivePowers.turn());
        out.put("manual", manual);
        out.put("fl", wheelPowers[FL]);
        out.put("fr", wheelPowers[FR]);
        out.put("bl", wheelPowers[BL]);
        out.put("br", wheelPowers[BR]);
        return out;
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

    private static double[] unnormalized(DrivePowers powers) {
        double forward = powers.forward();
        double strafe = powers.strafe();
        double turn = powers.turn();
        return new double[] {
                forward - strafe - turn,
                forward + strafe + turn,
                forward + strafe - turn,
                forward - strafe + turn};
    }

    private static double clip(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    private static double clip01(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
