package boobuzz.core.subsystem.turret;

import boobuzz.core.contract.ActionValidator;
import boobuzz.core.contract.Event;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.ITurret.AimResult;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.HashMap;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

public class PairedCrTurretTest {

    private static final String A = RobotConstants.TURRET_PRIMARY_SERVO_NAME;
    private static final String B = RobotConstants.TURRET_SECONDARY_SERVO_NAME;
    private static final String ENC = RobotConstants.TURRET_ENCODER_NAME;
    private static final String ANALOG = RobotConstants.TURRET_ANALOG_NAME;
    private static final double TICKS_PER_TURRET_REV = 0.715 * 8192.0;

    @Test
    public void seedOneAnalogIsZeroDegrees() {
        assertEquals(0.0, PairedCrTurret.analogAngleDeg(1.1458333333), 1e-6);
        assertEquals(90.0, PairedCrTurret.analogAngleDeg(volts(90.0)), 1e-9);
        assertEquals(-90.0, PairedCrTurret.analogAngleDeg(volts(-90.0)), 1e-9);
        assertNull(PairedCrTurret.analogAngleDeg(null));
        assertNull(PairedCrTurret.analogAngleDeg(Double.NaN));
        assertNull(PairedCrTurret.analogAngleDeg(-0.01));
        assertNull(PairedCrTurret.analogAngleDeg(3.31));
    }

    @Test
    public void encoderSignAndGearMatchArchive() {
        // Encoder reversed; 0.715 encoder turns per turret turn.
        assertEquals(360.0, PairedCrTurret.ticksToDegrees(-TICKS_PER_TURRET_REV), 1e-9);
        assertEquals(-90.0, PairedCrTurret.ticksToDegrees(TICKS_PER_TURRET_REV / 4.0), 1e-9);
    }

    @Test
    public void calibratesForTwoSecondsWithBothOutputsZero() {
        Plant plant = new Plant(0.0);
        PairedCrTurret turret = new PairedCrTurret();
        long t = 0;
        for (; t < 2000; t += 20) {
            RobotAction action = plant.step(turret, t);
            assertEquals(0.0, action.motor(A), 0.0);
            assertEquals(0.0, action.motor(B), 0.0);
            assertEquals(AimResult.NOT_INITIALIZED, turret.aimRelative(Math.toRadians(45.0)));
            assertFalse(turret.isInitialized());
        }
        RobotAction ready = plant.step(turret, 2000);
        assertTrue(turret.isInitialized());
        assertEquals(1, count(ready, "turret.ready"));
        assertEquals(0.0, turret.angleDeg(), 1e-9);
    }

    @Test
    public void startupInvalidAnalogPreventsAimUntilValid() {
        Plant plant = new Plant(0.0);
        plant.analogOverride = null;
        plant.analogMissing = true;
        PairedCrTurret turret = new PairedCrTurret();
        RobotAction first = plant.step(turret, 0);
        assertEquals(1, count(first, "turret.fault"));
        for (long t = 20; t <= 3000; t += 20) {
            RobotAction action = plant.step(turret, t);
            assertEquals(0, count(action, "turret.fault"));
            assertEquals(0.0, action.motor(A), 0.0);
        }
        assertEquals(AimResult.NOT_INITIALIZED, turret.aimRelative(0.0));

        // 3.3 V decodes to 328.7 deg: an unreachable initial angle is invalid too.
        plant.analogMissing = false;
        plant.analogOverride = 3.3;
        plant.step(turret, 3020);
        assertFalse(turret.isCalibrating());

        plant.analogOverride = null;
        plant.step(turret, 3040);
        assertTrue(turret.isCalibrating());
        plant.run(turret, 3060, 5040);
        assertTrue(turret.isInitialized());
    }

    @Test
    public void snapToZeroIsDisabled() {
        Plant plant = new Plant(10.0);
        PairedCrTurret turret = new PairedCrTurret();
        plant.run(turret, 0, 2000);
        assertTrue(turret.isInitialized());
        assertEquals(10.0, turret.angleDeg(), 1e-6);
    }

    @Test
    public void rangeValidation() {
        PairedCrTurret turret = readyTurret(new Plant(0.0));
        assertEquals(AimResult.ACCEPTED, turret.aimRelative(Math.toRadians(45.0)));
        assertEquals(AimResult.ACCEPTED, turret.aimRelative(Math.toRadians(-45.0)));
        assertEquals(AimResult.ACCEPTED, turret.aimRelative(Math.toRadians(90.0)));
        assertEquals(AimResult.ACCEPTED, turret.aimRelative(Math.toRadians(-90.0)));
        assertEquals(-90.0, turret.targetDeg(), 1e-9);
        assertEquals(AimResult.OUT_OF_RANGE, turret.aimRelative(Math.toRadians(91.0)));
        assertEquals("rejected aim keeps the previous target", -90.0, turret.targetDeg(), 1e-9);
        assertEquals(AimResult.INVALID_INPUT, turret.aimRelative(Double.NaN));
        assertEquals(AimResult.INVALID_INPUT, turret.aimStatus());
        assertFalse(turret.onTarget());
    }

    @Test
    public void reachesPlusMinus45WithMatchedOutputsAndLocks() {
        for (double goal : new double[] {45.0, -45.0}) {
            Plant plant = new Plant(0.0);
            PairedCrTurret turret = readyTurret(plant);
            assertEquals(AimResult.ACCEPTED, turret.aimRelative(Math.toRadians(goal)));
            int locked = 0;
            long lockedAt = -1;
            for (long t = 2020; t <= 5000; t += 20) {
                RobotAction action = plant.step(turret, t);
                assertEquals(action.motor(A), action.motor(B), 0.0);
                ActionValidator.validate(action, Mechanism.DEFAULT);
                assertFalse("turret never writes shooter power",
                        action.motors().containsKey(RobotConstants.SHOOTER_LEFT_MOTOR_NAME));
                locked += count(action, "turret.locked");
                if (lockedAt < 0 && turret.onTarget()) {
                    lockedAt = t;
                }
                turret.aimRelative(Math.toRadians(goal));
            }
            assertEquals(1, locked);
            assertTrue("locks within 2 s, got " + lockedAt, lockedAt > 0 && lockedAt < 4020);
            assertEquals(goal, plant.angleDeg, RobotConstants.TURRET_SETTLE_TOL_DEG);
            assertEquals(goal, turret.angleDeg(), RobotConstants.TURRET_SETTLE_TOL_DEG);
        }
    }

    @Test
    public void hardLimitTargetNeverPenetratesStop() {
        Plant plant = new Plant(80.0);
        PairedCrTurret turret = readyTurret(plant);
        turret.aimRelative(Math.toRadians(90.0));
        for (long t = 2020; t <= 5000; t += 20) {
            plant.step(turret, t);
            assertTrue("angle " + plant.angleDeg, plant.angleDeg <= RobotConstants.TURRET_MAX_DEG);
        }
        assertEquals(0.0, PairedCrTurret.softLimit(0.5, 90.0), 0.0);
        assertEquals(0.25, PairedCrTurret.softLimit(0.5, 87.5), 1e-9);
        assertEquals(-0.5, PairedCrTurret.softLimit(-0.5, 90.0), 0.0);
        assertEquals(0.0, PairedCrTurret.softLimit(-0.5, -91.0), 0.0);
    }

    @Test
    public void fieldAimUsesBearingMinusHeading() {
        assertEquals(45.0, PairedCrTurret.relativeBearingDeg(new Pose(0, 0, 0), 10, 10), 1e-9);
        assertEquals(-45.0, PairedCrTurret.relativeBearingDeg(
                new Pose(0, 0, Math.PI / 2), 10, 10), 1e-9);

        Plant plant = new Plant(0.0);
        PairedCrTurret turret = readyTurret(plant);
        turret.aimAt(10.0, 10.0);
        assertEquals(AimResult.ACCEPTED, turret.aimStatus());
        assertEquals(45.0, turret.targetDeg(), 1e-9);
        turret.aimAt(-10.0, 0.0);
        assertEquals(AimResult.OUT_OF_RANGE, turret.aimStatus());
        assertEquals(45.0, turret.targetDeg(), 1e-9);
        turret.aimAt(Double.NaN, 0.0);
        assertEquals(AimResult.INVALID_INPUT, turret.aimStatus());

        // Tracking re-evaluates as the robot turns: heading 30 deg -> 15 deg relative.
        turret.aimAt(10.0, 10.0);
        plant.heading = Math.toRadians(30.0);
        plant.step(turret, 2020);
        assertEquals(15.0, turret.targetDeg(), 1e-9);
    }

    @Test
    public void holdKeepsTargetAndPendingHoldHoldsCurrentAngle() {
        Plant plant = new Plant(20.0);
        PairedCrTurret turret = new PairedCrTurret();
        plant.step(turret, 0);
        turret.hold();
        plant.run(turret, 20, 2000);
        assertTrue(turret.isInitialized());
        assertEquals("pending hold freezes the calibrated angle", 20.0, turret.targetDeg(), 1e-6);

        turret.aimRelative(Math.toRadians(40.0));
        plant.run(turret, 2020, 2100);
        turret.hold();
        double frozen = turret.targetDeg();
        assertEquals(40.0, frozen, 1e-9);
        plant.run(turret, 2120, 2400);
        turret.hold();
        assertEquals("repeated hold keeps the same target", frozen, turret.targetDeg(), 0.0);
    }

    @Test
    public void disableZerosBothAndDoesNotRestart() {
        Plant plant = new Plant(0.0);
        PairedCrTurret turret = readyTurret(plant);
        turret.aimRelative(Math.toRadians(45.0));
        RobotAction moving = plant.step(turret, 2020);
        assertTrue(moving.motor(A) > 0.0);
        turret.disable();
        for (long t = 2040; t <= 3000; t += 20) {
            RobotAction action = plant.step(turret, t);
            assertEquals(0.0, action.motor(A), 0.0);
            assertEquals(0.0, action.motor(B), 0.0);
        }
        assertFalse(turret.onTarget());
        assertEquals(AimResult.ACCEPTED, turret.aimRelative(Math.toRadians(45.0)));
        assertTrue(plant.step(turret, 3020).motor(A) != 0.0);
    }

    @Test
    public void encoderLossZerosAndRecalibrates() {
        Plant plant = new Plant(0.0);
        PairedCrTurret turret = readyTurret(plant);
        turret.aimRelative(Math.toRadians(30.0));
        plant.run(turret, 2020, 2200);
        plant.encoderMissing = true;
        RobotAction lost = plant.step(turret, 2220);
        assertEquals(0.0, lost.motor(A), 0.0);
        assertEquals(0.0, lost.motor(B), 0.0);
        assertEquals(1, count(lost, "turret.fault"));
        assertFalse(turret.isInitialized());

        plant.encoderMissing = false;
        plant.step(turret, 2240);
        assertTrue(turret.isCalibrating());
        plant.run(turret, 2260, 4240);
        assertTrue(turret.isInitialized());
        assertEquals("recalibrated from the analog", plant.angleDeg, turret.angleDeg(), 0.5);
        assertEquals("a fixed aim degrades to hold after a fault", AimResult.ACCEPTED,
                turret.aimStatus());
    }

    @Test
    public void encoderJumpLargerThanRangeRecalibrates() {
        Plant plant = new Plant(0.0);
        PairedCrTurret turret = readyTurret(plant);
        plant.encoderOffset = 10000;
        RobotAction jump = plant.step(turret, 2020);
        assertEquals(1, count(jump, "turret.fault"));
        assertTrue(turret.isCalibrating());
        assertEquals(0.0, jump.motor(A), 0.0);
    }

    @Test
    public void midCalibrationAnalogLossRestartsCalibration() {
        Plant plant = new Plant(0.0);
        PairedCrTurret turret = new PairedCrTurret();
        plant.run(turret, 0, 1000);
        plant.analogMissing = true;
        plant.step(turret, 1020);
        assertFalse(turret.isCalibrating());
        plant.analogMissing = false;
        plant.step(turret, 1040);
        plant.run(turret, 1060, 3020);
        assertTrue(turret.isCalibrating());
        plant.step(turret, 3040);
        assertTrue(turret.isInitialized());
    }

    private static PairedCrTurret readyTurret(Plant plant) {
        PairedCrTurret turret = new PairedCrTurret();
        plant.run(turret, 0, 2000);
        assertTrue(turret.isInitialized());
        return turret;
    }

    private static double volts(double turretDeg) {
        return (125.0 + turretDeg * 0.715) / 360.0 * 3.3;
    }

    private static int count(RobotAction action, String name) {
        int n = 0;
        for (Event event : action.events()) {
            if (event.name().equals(name)) {
                n++;
            }
        }
        return n;
    }

    /**
     * Toy first-order plant only to close the loop in tests: rate = mean CR power times
     * 300 deg/s, hard stops at +-90 deg, encoder reversed with archive gearing.
     */
    private static final class Plant {
        double angleDeg;
        double heading;
        boolean analogMissing;
        boolean encoderMissing;
        Double analogOverride;
        int encoderOffset;
        private long lastT = -1;
        private double lastPower;

        Plant(double angleDeg) {
            this.angleDeg = angleDeg;
        }

        RobotAction step(PairedCrTurret turret, long t) {
            if (lastT >= 0) {
                angleDeg += lastPower * 300.0 * (t - lastT) / 1000.0;
                angleDeg = Math.max(-90.0, Math.min(90.0, angleDeg));
            }
            lastT = t;
            Map<String, Integer> enc = new HashMap<>();
            if (!encoderMissing) {
                enc.put(ENC, (int) Math.round(-angleDeg / 360.0 * TICKS_PER_TURRET_REV)
                        + encoderOffset);
            }
            Map<String, Double> analog = new HashMap<>();
            if (!analogMissing) {
                analog.put(ANALOG, analogOverride != null ? analogOverride : volts(angleDeg));
            }
            turret.observe(new RobotState(t, enc, Map.of(), heading,
                    new Pose(0.0, 0.0, heading), 12.0, analog));
            RobotAction.Builder out = new RobotAction.Builder();
            turret.update(out);
            RobotAction action = out.build();
            lastPower = (action.motor(A) + action.motor(B)) / 2.0;
            return action;
        }

        void run(PairedCrTurret turret, long from, long to) {
            for (long t = from; t <= to; t += 20) {
                step(turret, t);
            }
        }
    }
}
