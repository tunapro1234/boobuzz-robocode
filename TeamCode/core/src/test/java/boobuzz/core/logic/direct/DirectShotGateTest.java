package boobuzz.core.logic.direct;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.logic.MechanismProfile;
import boobuzz.core.subsystem.IDrive;
import boobuzz.core.subsystem.ITurret;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.intake.PowerIntake;
import boobuzz.core.subsystem.shooter.FlywheelShooter;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Collections;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

/**
 * Direct engine, REAL profile: preset shots honor the turret aim result and wait for the
 * hood and turret before feeding; STOP_SHOOTING spins the flywheel down (review B08).
 */
public class DirectShotGateTest {

    private static final String FEEDER = RobotConstants.FEEDER_MOTOR_NAME;
    private static final String FLYWHEEL = RobotConstants.SHOOTER_RIGHT_MOTOR_NAME;
    private static final double TICKS_AT_4000 = 1166.6666667;

    @Test
    public void rejectedAimRejectsThePresetShot() {
        Rig rig = new Rig();
        rig.turret.status = ITurret.AimResult.OUT_OF_RANGE;
        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        RequestStatus rejected = rig.status(2);
        assertEquals(RequestStatus.State.REJECTED, rejected.state());
        assertEquals("turret: OUT_OF_RANGE", rejected.note());
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
    }

    @Test
    public void presetShotWaitsForHoodAndTurret() {
        Rig rig = new Rig();
        rig.turret.onTarget = false;
        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        long commanded = rig.t;
        boolean sawHood = false;
        for (int i = 0; i < 150; i++) {
            rig.run(RequestBatch.idle());
            assertEquals(0.0, rig.last.motor(FEEDER), 0.0);
            sawHood |= "hood settling".equals(rig.status(2).note());
        }
        assertTrue("waited for the hood", sawHood);
        assertEquals("aiming", rig.status(2).note());

        rig.turret.onTarget = true;
        long fedAt = -1;
        for (int i = 0; i < 100 && fedAt < 0; i++) {
            rig.run(RequestBatch.idle());
            if (rig.last.motor(FEEDER) > 0.0) {
                fedAt = rig.t;
            }
        }
        long hoodSettle = Math.round((RobotConstants.HOOD_MAX_DEG - RobotConstants.HOOD_MIN_DEG)
                / RobotConstants.HOOD_FIXTURE_RATE_DEG_S * 1000.0)
                + RobotConstants.HOOD_SETTLE_MARGIN_MS;
        assertTrue("fed once on target", fedAt > 0);
        assertTrue(fedAt - commanded >= hoodSettle);
        assertEquals(RequestStatus.State.DONE, rig.runUntilTerminal(2).state());
    }

    @Test
    public void neverInitializedTurretFailsAfterTheStartupBound() {
        Rig rig = new Rig();
        rig.turret.status = ITurret.AimResult.NOT_INITIALIZED;
        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        long began = rig.t;
        RequestStatus failed = rig.runUntilTerminal(2);
        assertEquals(RequestStatus.State.FAILED, failed.state());
        assertEquals("prepare timeout: aiming: NOT_INITIALIZED", failed.note());
        long limit = RobotConstants.SHOT_TURRET_STARTUP_BOUND_MS
                + RobotConstants.SHOT_PREPARE_TIMEOUT_MS;
        assertTrue(rig.t - began > limit);
        assertTrue(rig.t - began <= limit + 40);
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
        assertEquals(0, rig.feederPulses);
    }

    @Test
    public void turretNeverOnTargetFailsAfterThePrepareTimeout() {
        Rig rig = new Rig();
        rig.turret.onTarget = false;
        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        long began = rig.t;
        RequestStatus failed = rig.runUntilTerminal(2);
        assertEquals(RequestStatus.State.FAILED, failed.state());
        assertEquals("prepare timeout: aiming", failed.note());
        assertTrue(rig.t - began > RobotConstants.SHOT_PREPARE_TIMEOUT_MS);
        assertTrue(rig.t - began <= RobotConstants.SHOT_PREPARE_TIMEOUT_MS + 40);
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
    }

    @Test
    public void turretRecalibratingBetweenPulsesIsReaimedAndTheShotFinishes() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(2, 2)));
        while (rig.feederPulses == 0) {
            rig.run(RequestBatch.idle());
        }
        // The real turret drops to NOT_INITIALIZED on a fault and recalibrates.
        rig.turret.status = ITurret.AimResult.NOT_INITIALIZED;
        rig.turret.current = ITurret.AimResult.NOT_INITIALIZED;
        for (int i = 0; i < RobotConstants.SHOT_TURRET_STARTUP_BOUND_MS / 20; i++) {
            rig.run(RequestBatch.idle());
            assertEquals(RequestStatus.State.ACTIVE, rig.status(2).state());
        }
        assertEquals(1, rig.feederPulses);
        rig.turret.status = ITurret.AimResult.ACCEPTED;
        RequestStatus done = rig.runUntilTerminal(2);
        assertEquals(done.note(), RequestStatus.State.DONE, done.state());
        assertEquals(2, rig.feederPulses);
        assertEquals(ITurret.AimResult.ACCEPTED, rig.turret.aimStatus());
    }

    @Test
    public void turretAimRetargetsAPresetShot() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(2, 2)));
        rig.run(RequestBatch.of(Request.turretAim(3, 48.0, 96.0)));
        assertEquals(RequestStatus.State.DONE, rig.status(3).state());
        RequestStatus cancelled = rig.status(2);
        assertEquals(RequestStatus.State.REJECTED, cancelled.state());
        assertEquals("turret retargeted", cancelled.note());
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
    }

    @Test
    public void presetChangeAppliesFromTheNextPulse() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(2, 2)));
        while (rig.last.motor(FEEDER) == 0.0) {
            rig.run(RequestBatch.idle());
        }
        double first = rig.shooter.targetRpm();
        rig.run(RequestBatch.of(Request.setShotPreset(3, 3000.0, 40.0, 0.1)));
        assertEquals(RequestStatus.State.DONE, rig.status(3).state());
        while (rig.last.motor(FEEDER) > 0.0) {
            assertEquals("the running pulse keeps its rpm", first, rig.shooter.targetRpm(), 0.0);
            rig.run(RequestBatch.idle());
        }
        rig.run(RequestBatch.idle());
        assertEquals(3000.0, rig.shooter.targetRpm(), 0.0);
        assertEquals(0.1, rig.turret.lastRelative, 0.0);
        assertEquals(RequestStatus.State.DONE, rig.runUntilTerminal(2).state());
    }

    @Test
    public void stopShootingDuringAJamClearLeavesTheRecoveryInCharge() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.mechanismRecovery(2, 2)));
        rig.run(RequestBatch.of(Request.stopShooting(3)));
        assertEquals(RequestStatus.State.DONE, rig.status(3).state());
        assertEquals(RequestStatus.State.ACTIVE, rig.status(2).state());
        assertEquals("jam clear keeps driving the flywheel", 1.0,
                rig.last.motor(FLYWHEEL), 0.0);
    }

    @Test
    public void turretFinishingStartupInsideTheBoundLetsTheShotRun() {
        Rig rig = new Rig();
        rig.turret.status = ITurret.AimResult.NOT_INITIALIZED;
        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        for (int i = 0; i < RobotConstants.SHOT_TURRET_STARTUP_BOUND_MS / 40; i++) {
            rig.run(RequestBatch.idle());
            assertEquals(RequestStatus.State.ACTIVE, rig.status(2).state());
        }
        rig.turret.status = ITurret.AimResult.ACCEPTED;
        RequestStatus done = rig.runUntilTerminal(2);
        assertEquals(done.note(), RequestStatus.State.DONE, done.state());
    }

    @Test
    public void stopShootingSpinsDownAWarmFlywheel() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        assertEquals(RequestStatus.State.DONE, rig.runUntilTerminal(2).state());
        rig.run(RequestBatch.idle());
        assertTrue("warm after the shot", rig.last.motor(FLYWHEEL) > 0.0);
        rig.run(RequestBatch.of(Request.stopShooting(3)));
        assertEquals(RequestStatus.State.DONE, rig.status(3).state());
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
    }

    @Test
    public void stopShootingFinishesTheShotThenSpinsDown() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(2, 3)));
        while (rig.last.motor(FEEDER) == 0.0) {
            rig.run(RequestBatch.idle());
        }
        rig.run(RequestBatch.of(Request.stopShooting(3)));
        assertTrue("the running pulse is not cut", rig.last.motor(FEEDER) > 0.0);
        RequestStatus done = rig.runUntilTerminal(2);
        assertEquals(RequestStatus.State.DONE, done.state());
        assertEquals("stopped", done.note());
        assertEquals(1.0 / 3.0, done.progress(), 1e-9);
        assertEquals(1, rig.feederPulses);
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
    }

    @Test
    public void stopShootingEndsASpinUp() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.spinUp(2, 3000.0)));
        rig.run(RequestBatch.of(Request.stopShooting(3)));
        RequestStatus stopped = rig.status(2);
        assertEquals(RequestStatus.State.DONE, stopped.state());
        assertEquals("stopped", stopped.note());
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
    }

    @Test
    public void aRequestCancelledInItsOwnBatchNeverStarts() {
        Rig rig = new Rig();
        rig.run(new RequestBatch(RequestStream.idle(), List.of(Request.shoot(2, 1)),
                new int[] {2}));
        RequestStatus cancelled = rig.status(2);
        assertEquals(RequestStatus.State.REJECTED, cancelled.state());
        assertEquals("cancelled", cancelled.note());
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
    }

    private static final class Rig {
        final FlywheelShooter shooter = new FlywheelShooter();
        final ScriptedTurret turret = new ScriptedTurret();
        final DirectEngine engine;
        final Pose pose = new Pose(24.0, 48.0, 0.0);
        long t = -20;
        int feederPulses;
        RobotAction last = RobotAction.zero();
        List<RequestStatus> statuses = Collections.emptyList();

        Rig() {
            engine = new DirectEngine(new Subsystems(new FixedDrive(pose), shooter,
                    new PowerIntake(), turret), MechanismProfile.REAL);
            run(RequestBatch.idle());
        }

        void run(RequestBatch batch) {
            t += 20;
            // Ideal flywheel: the measured speed is the commanded target.
            Map<String, Double> vel = Map.of(RobotConstants.SHOOTER_FEEDBACK_ENCODER_NAME,
                    shooter.targetRpm() * TICKS_AT_4000 / 4000.0);
            engine.sense(new RobotState(t, Map.of(), vel, pose.heading(), pose, 12.0));
            engine.act(batch);
            boolean wasFeeding = last.motor(FEEDER) > 0.0;
            last = engine.action();
            if (!wasFeeding && last.motor(FEEDER) > 0.0) {
                feederPulses++;
            }
            statuses = engine.drainStatuses();
        }

        RequestStatus status(int id) {
            RequestStatus found = null;
            for (RequestStatus status : statuses) {
                if (status.id() == id) {
                    found = status;
                }
            }
            assertNotNull("no status for " + id + " at t=" + t, found);
            return found;
        }

        RequestStatus runUntilTerminal(int id) {
            for (int i = 0; i < 1000; i++) {
                for (RequestStatus status : statuses) {
                    if (status.id() == id && status.state() != RequestStatus.State.ACTIVE) {
                        return status;
                    }
                }
                run(RequestBatch.idle());
            }
            throw new AssertionError("request " + id + " never finished");
        }
    }

    private static final class ScriptedTurret implements ITurret {
        boolean onTarget = true;
        AimResult status = AimResult.ACCEPTED;
        AimResult current = AimResult.NOT_INITIALIZED;
        double lastRelative = Double.NaN;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void setRobotPose(Pose pose) {}
        @Override public void aimAt(double fieldX, double fieldY) {}
        @Override public AimResult aimRelative(double angleRad) {
            lastRelative = angleRad;
            current = status;
            return current;
        }
        @Override public AimResult aimStatus() { return current; }
        @Override public void scan() {}
        @Override public void hold() {}
        @Override public void disable() {}
        @Override public boolean onTarget() { return onTarget && current == AimResult.ACCEPTED; }
        @Override public double angleRad() { return 0.0; }
    }

    private record FixedDrive(Pose pose) implements IDrive {
        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void manual(double vx, double vy, double omega) {}
        @Override public void follow(PathRequest request) {}
        @Override public void stop() {}
        @Override public boolean pathDone() { return true; }
    }
}
