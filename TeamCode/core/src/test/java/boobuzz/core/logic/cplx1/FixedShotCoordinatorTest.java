package boobuzz.core.logic.cplx1;

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
import boobuzz.core.subsystem.hood.PairedHood;
import boobuzz.core.subsystem.intake.PowerIntake;
import boobuzz.core.subsystem.shooter.FlywheelShooter;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

/**
 * Spec B08: the shared shot coordinator over the real FlywheelShooter (feeder + hood)
 * and PowerIntake, with an ideal flywheel plant (measured = commanded) and a scripted
 * turret, so the trace pins coordinator behavior and not plant dynamics.
 */
public class FixedShotCoordinatorTest {

    private static final String FEEDER = RobotConstants.FEEDER_MOTOR_NAME;
    private static final String INTAKE = RobotConstants.INTAKE_MOTOR_NAME;
    private static final String FLYWHEEL = RobotConstants.SHOOTER_RIGHT_MOTOR_NAME;
    private static final double TICKS_AT_4000 = 1166.6666667;

    @Test
    public void realProfileCountOnlyShotUsesArchivePreset() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(1, 1)));
        assertEquals(4000.0, rig.shooter.targetRpm(), 0.0);
        rig.run(RequestBatch.idle());
        assertEquals(45.0, rig.engine.shooter().lastHood(), 0.0);
        assertEquals(0.0, rig.turret.lastRelative, 0.0);
        assertEquals(1.0 - PairedHood.servoFraction(45.0),
                rig.last.servos().get(RobotConstants.HOOD_LEFT_SERVO_NAME), 1e-12);
        assertEquals(RequestStatus.State.DONE, rig.runUntilTerminal(1).state());
    }

    @Test
    public void ownershipTraceIntakeFeederPulseAndGap() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.intakeOn(1, 1.0)));
        assertEquals(1.0, rig.last.motor(INTAKE), 0.0);

        rig.run(RequestBatch.of(Request.shoot(2, 3)));
        assertEquals("the shot owns the intake from its first tick",
                RobotConstants.SHOOT_INTAKE_POWER, rig.last.motor(INTAKE), 0.0);
        int start = rig.trace.size() - 1;
        RequestStatus done = rig.runUntilTerminal(2);
        assertEquals(RequestStatus.State.DONE, done.state());
        int end = rig.trace.size() - 1;

        for (int i = start; i < end; i++) {
            assertEquals("t=" + rig.trace.get(i).t, RobotConstants.SHOOT_INTAKE_POWER,
                    rig.trace.get(i).intake, 0.0);
        }
        rig.run(RequestBatch.idle());
        assertEquals("release restores the manual demand", 1.0, rig.last.motor(INTAKE), 0.0);

        List<long[]> pulses = pulses(rig.trace);
        assertEquals(3, pulses.size());
        for (long[] pulse : pulses) {
            assertTrue("pulse " + (pulse[1] - pulse[0]) + " ms",
                    pulse[1] - pulse[0] >= RobotConstants.FEEDER_PULSE_MS);
        }
        for (int i = 1; i < pulses.size(); i++) {
            long gap = pulses.get(i)[0] - pulses.get(i - 1)[1];
            // Archive requestPulseAndDelay: the motor restarts at the first 20 ms tick at or
            // after the delay, counted from the tick that stopped it.
            assertEquals("gap", RobotConstants.FEEDER_POST_PULSE_DELAY_MS, gap);
        }
    }

    @Test
    public void releaseWithoutManualDemandLeavesIntakeOff() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        assertEquals(RobotConstants.SHOOT_INTAKE_POWER, rig.last.motor(INTAKE), 0.0);
        rig.runUntilTerminal(2);
        rig.run(RequestBatch.idle());
        assertEquals(0.0, rig.last.motor(INTAKE), 0.0);
    }

    @Test
    public void sameTickCancelStopsFeederFlywheelAndRestoresIntake() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.intakeOn(1, 1.0)));
        rig.run(RequestBatch.of(Request.shoot(2, 3)));
        rig.runUntilFeeding();

        rig.run(new RequestBatch(RequestStream.idle(), List.of(), new int[] {2}));
        assertEquals(0.0, rig.last.motor(FEEDER), 0.0);
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
        assertEquals(1.0, rig.last.motor(INTAKE), 0.0);
        assertEquals(RequestStatus.State.REJECTED, rig.status(2).state());
        assertEquals(ShooterLogic.State.IDLE, rig.engine.shooter().state());
    }

    @Test
    public void sameTickCancelAllStopsEverythingAndDisablesTurret() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.intakeOn(1, 1.0)));
        rig.run(RequestBatch.of(Request.shoot(2, 3)));
        rig.runUntilFeeding();

        rig.run(RequestBatch.cancelAll());
        assertEquals(0.0, rig.last.motor(FEEDER), 0.0);
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
        assertEquals(0.0, rig.last.motor(INTAKE), 0.0);
        assertTrue(rig.turret.disableCalls > 0);
        assertEquals(RequestStatus.State.REJECTED, rig.status(2).state());
    }

    @Test
    public void feedWaitsForHoodTurretAndStationaryChassis() {
        Rig rig = new Rig();
        rig.turret.onTarget = false;
        rig.pose = new Pose(24.0, 48.0, 0.0);
        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        List<String> notes = new ArrayList<>();
        // Stay inside the 3 s prepare timeout for the whole blocked sequence.
        for (int i = 0; i < 60; i++) {
            rig.run(RequestBatch.idle());
            notes.add(rig.status(2).note());
            assertEquals(0.0, rig.last.motor(FEEDER), 0.0);
        }
        assertTrue(notes.toString(), notes.contains("hood settling"));
        assertEquals("aiming", notes.get(notes.size() - 1));

        // 6 in/s: above the 2 in/s stationary limit. The 200 ms window needs a few ticks
        // to see the motion, so the drive starts before the turret locks.
        for (int i = 0; i < 40; i++) {
            if (i == 20) {
                rig.turret.onTarget = true;
            }
            rig.pose = new Pose(rig.pose.x() + 0.12, 48.0, 0.0);
            rig.run(RequestBatch.idle());
            assertEquals(0.0, rig.last.motor(FEEDER), 0.0);
        }
        assertEquals("chassis moving", rig.status(2).note());

        long stoppedAt = rig.t;
        rig.runUntilFeeding();
        assertTrue("stationary hold " + (rig.t - stoppedAt) + " ms",
                rig.t - stoppedAt >= RobotConstants.STATIONARY_HOLD_MS);
    }

    @Test
    public void hoodSettleGatesTheFirstPulse() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        long shotAt = rig.t;
        rig.runUntilFeeding();
        // Unknown start position: full 25 deg travel at the fixture rate plus the margin.
        long settle = Math.round((RobotConstants.HOOD_MAX_DEG - RobotConstants.HOOD_MIN_DEG)
                / RobotConstants.HOOD_FIXTURE_RATE_DEG_S * 1000.0) + RobotConstants.HOOD_SETTLE_MARGIN_MS;
        assertTrue("fed " + (rig.t - shotAt) + " ms after the hood command",
                rig.t - shotAt >= settle);
    }

    @Test
    public void prepareTimeoutStartsOnlyAfterTurretStartup() {
        Rig rig = new Rig();
        rig.turret.status = ITurret.AimResult.NOT_INITIALIZED;
        rig.turret.onTarget = false;
        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        // Startup ends inside the bound: the whole prepare timeout still follows it.
        long startupTicks = RobotConstants.SHOT_TURRET_STARTUP_BOUND_MS / 20 - 5;
        for (int i = 0; i < startupTicks; i++) {
            rig.run(RequestBatch.idle());
            assertEquals(RequestStatus.State.ACTIVE, rig.status(2).state());
        }
        assertEquals("aiming: NOT_INITIALIZED", rig.status(2).note());

        rig.turret.status = ITurret.AimResult.ACCEPTED;
        long startupDone = rig.t + 20;
        RequestStatus failed = rig.runUntilTerminal(2);
        assertEquals(RequestStatus.State.FAILED, failed.state());
        assertEquals("prepare timeout: aiming", failed.note());
        assertTrue(rig.t - startupDone > RobotConstants.SHOT_PREPARE_TIMEOUT_MS);
        assertTrue(rig.t - startupDone <= RobotConstants.SHOT_PREPARE_TIMEOUT_MS + 40);
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
        assertEquals(ShooterLogic.State.FAULT, rig.engine.shooter().state());
    }

    @Test
    public void neverInitializedTurretFailsAfterTheStartupBound() {
        Rig rig = new Rig();
        rig.turret.status = ITurret.AimResult.NOT_INITIALIZED;
        rig.turret.onTarget = false;
        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        long began = rig.t;
        RequestStatus failed = rig.runUntilTerminal(2);
        assertEquals(RequestStatus.State.FAILED, failed.state());
        assertEquals("prepare timeout: aiming: NOT_INITIALIZED", failed.note());
        long waited = rig.t - began;
        long limit = RobotConstants.SHOT_TURRET_STARTUP_BOUND_MS
                + RobotConstants.SHOT_PREPARE_TIMEOUT_MS;
        assertTrue("failed after " + waited + " ms", waited > limit);
        assertTrue("failed after " + waited + " ms", waited <= limit + 40);
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
    }

    @Test
    public void stopBeforeTurretStartupIsAccepted() {
        Rig rig = new Rig();
        rig.turret.status = ITurret.AimResult.NOT_INITIALIZED;
        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        rig.run(RequestBatch.of(Request.stopShooting(3)));
        assertEquals(RequestStatus.State.DONE, rig.status(3).state());
        RequestStatus stopped = rig.status(2);
        assertEquals(RequestStatus.State.DONE, stopped.state());
        assertEquals("stopped", stopped.note());
    }

    @Test
    public void stopShootingFinishesThePulseAndStartsNoNewOne() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(2, 3)));
        rig.runUntilFeeding();
        long pulseStart = rig.t;
        rig.run(RequestBatch.of(Request.stopShooting(3)));
        assertTrue("the running pulse is not cut", rig.last.motor(FEEDER) > 0.0);
        RequestStatus stopped = rig.runUntilTerminal(2);
        assertEquals(RequestStatus.State.DONE, stopped.state());
        assertEquals("stopped", stopped.note());
        assertTrue(rig.t - pulseStart >= RobotConstants.FEEDER_PULSE_MS);
        for (int i = 0; i < 50; i++) {
            rig.run(RequestBatch.idle());
        }
        assertEquals(1, pulses(rig.trace).size());
        assertEquals("archive: releasing RT disables the shooter", 0.0,
                rig.last.motor(FLYWHEEL), 0.0);
    }

    @Test
    public void countDoneKeepsTheFlywheelWarmUntilStopShooting() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        assertEquals(RequestStatus.State.DONE, rig.runUntilTerminal(2).state());
        for (int i = 0; i < 50; i++) {
            rig.run(RequestBatch.idle());
            assertTrue("warm between one-at-a-time shots", rig.last.motor(FLYWHEEL) > 0.0);
        }
        rig.run(RequestBatch.of(Request.stopShooting(3)));
        assertEquals(RequestStatus.State.DONE, rig.status(3).state());
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
        assertEquals(0.0, rig.shooter.targetRpm(), 0.0);
    }

    @Test
    public void stopShootingEndsASpinUp() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.spinUp(2, 3000.0)));
        assertEquals(RequestStatus.State.ACTIVE, rig.status(2).state());
        rig.run(RequestBatch.of(Request.stopShooting(3)));
        RequestStatus stopped = rig.status(2);
        assertEquals(RequestStatus.State.DONE, stopped.state());
        assertEquals("stopped", stopped.note());
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
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
    public void aRequestCancelledInItsOwnBatchNeverStarts() {
        Rig rig = new Rig();
        rig.run(new RequestBatch(RequestStream.idle(), List.of(Request.shoot(2, 1)),
                new int[] {2}));
        RequestStatus cancelled = rig.status(2);
        assertEquals(RequestStatus.State.REJECTED, cancelled.state());
        assertEquals("cancelled", cancelled.note());
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
        rig.run(RequestBatch.idle());
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
        assertEquals(0.0, rig.last.motor(FEEDER), 0.0);
    }

    @Test
    public void presetIsLatchedPerPulse() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(2, 2)));
        rig.runUntilFeeding();
        rig.run(RequestBatch.of(Request.setShotPreset(3, 3000.0, 40.0, 0.0)));
        assertEquals(RequestStatus.State.DONE, rig.status(3).state());
        while (rig.last.motor(FEEDER) > 0.0) {
            assertEquals("in-flight pulse keeps its latched rpm", 4000.0,
                    rig.shooter.targetRpm(), 0.0);
            rig.run(RequestBatch.idle());
        }
        rig.run(RequestBatch.idle());
        assertEquals(3000.0, rig.shooter.targetRpm(), 0.0);
        assertEquals(40.0, rig.engine.shooter().lastHood(), 0.0);
        assertEquals(RequestStatus.State.DONE, rig.runUntilTerminal(2).state());
    }

    @Test
    public void invalidPresetIsRejectedNotClamped() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.setShotPreset(3, 4000.0, 60.0, 0.0)));
        assertEquals(RequestStatus.State.REJECTED, rig.status(3).state());
        assertEquals(45.0, rig.engine.shooter().preset().hoodDeg(), 0.0);
    }

    @Test
    public void turretAimCancelsAPresetShot() {
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
    public void shootCountAboveThreeIsRejected() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(2, 4)));
        assertEquals(RequestStatus.State.REJECTED, rig.status(2).state());
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
    }

    // ---- harness -----------------------------------------------------------------------

    private static List<long[]> pulses(List<Sample> trace) {
        List<long[]> pulses = new ArrayList<>();
        long onAt = -1;
        for (Sample sample : trace) {
            if (sample.feeder > 0.0 && onAt < 0) {
                onAt = sample.t;
            } else if (sample.feeder == 0.0 && onAt >= 0) {
                pulses.add(new long[] {onAt, sample.t});
                onAt = -1;
            }
        }
        return pulses;
    }

    private record Sample(long t, double feeder, double intake) { }

    private static final class Rig {
        final FlywheelShooter shooter = new FlywheelShooter();
        final ScriptedTurret turret = new ScriptedTurret();
        final CplxEngine1 engine;
        final List<Sample> trace = new ArrayList<>();
        Pose pose = new Pose(24.0, 48.0, 0.0);
        long t = -20;
        RobotAction last;
        List<RequestStatus> statuses = Collections.emptyList();

        Rig() {
            engine = new CplxEngine1(new Subsystems(new FixedDrive(this), shooter,
                    new PowerIntake(), turret), MechanismProfile.REAL);
            // Let the stationary window fill before any request.
            for (int i = 0; i < 20; i++) {
                run(RequestBatch.idle());
            }
        }

        void run(RequestBatch batch) {
            t += 20;
            // Ideal flywheel: the measured speed is the commanded target.
            double rpm = shooter.targetRpm();
            Map<String, Double> vel = Map.of(RobotConstants.SHOOTER_FEEDBACK_ENCODER_NAME,
                    rpm * TICKS_AT_4000 / 4000.0);
            engine.sense(new RobotState(t, Map.of(), vel, pose.heading(), pose, 12.0));
            engine.act(batch);
            last = engine.action();
            statuses = engine.drainStatuses();
            trace.add(new Sample(t, last.motor(FEEDER), last.motor(INTAKE)));
        }

        /** Latest status for this id in the last tick. */
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

        void runUntilFeeding() {
            for (int i = 0; i < 1000; i++) {
                run(RequestBatch.idle());
                if (last.motor(FEEDER) > 0.0) {
                    return;
                }
            }
            throw new AssertionError("never fed");
        }
    }

    private static final class FixedDrive implements IDrive {
        private final Rig rig;

        FixedDrive(Rig rig) {
            this.rig = rig;
        }

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void manual(double vx, double vy, double omega) {}
        @Override public void follow(PathRequest request) {}
        @Override public void stop() {}
        @Override public boolean pathDone() { return true; }
        @Override public Pose pose() { return rig.pose; }
    }

    private static final class ScriptedTurret implements ITurret {
        boolean onTarget = true;
        AimResult status = AimResult.ACCEPTED;
        double lastRelative = Double.NaN;
        int disableCalls;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void setRobotPose(Pose pose) {}
        @Override public void aimAt(double fieldX, double fieldY) {}
        @Override public AimResult aimRelative(double angleRad) {
            lastRelative = angleRad;
            return status;
        }
        @Override public AimResult aimStatus() { return status; }
        @Override public void scan() {}
        @Override public void hold() {}
        @Override public void disable() { disableCalls++; }
        @Override public boolean onTarget() { return onTarget && status == AimResult.ACCEPTED; }
        @Override public double angleRad() { return 0.0; }
    }
}
