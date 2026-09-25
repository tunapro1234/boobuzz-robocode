package boobuzz.core.logic.cplx1;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RequestType;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.logic.MechanismProfile;
import boobuzz.core.logic.shot.MechanismRecovery;
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
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

/**
 * MECHANISM_RECOVERY through CplxEngine1 over the real FlywheelShooter and PowerIntake.
 * The flywheel plant follows the commanded sign so the spin-down-before-reverse guard is
 * observable: a driven closed loop reaches its target at once, open loop reaches power * 4000 RPM
 * at once, and zero power coasts toward 0 by {@link #COAST_STEP_RPM} per tick.
 */
public class MechanismRecoveryEngineTest {

    private static final String FEEDER = RobotConstants.FEEDER_MOTOR_NAME;
    private static final String INTAKE = RobotConstants.INTAKE_MOTOR_NAME;
    private static final String FLYWHEEL = RobotConstants.SHOOTER_RIGHT_MOTOR_NAME;
    private static final double TICKS_AT_4000 = 1166.6666667;
    /** Test fixture, not a measured coast rate: 4000 RPM coasts to rest in four ticks. */
    private static final double COAST_STEP_RPM = 1000.0;
    /** Test fixture: open-loop speed at full power. */
    private static final double OPEN_LOOP_RPM_AT_FULL_POWER = 4000.0;

    @Test
    public void reverseRunsIntakeAndFeederBackwardAndLeavesTheFlywheelAlone() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.intakeOn(1, 1.0)));
        rig.run(RequestBatch.of(Request.mechanismRecovery(2, 1)));
        assertEquals(-1.0, rig.last.motor(INTAKE), 0.0);
        assertEquals(-1.0, rig.last.motor(FEEDER), 0.0);
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
        assertEquals("reversing", rig.status(2).note());

        rig.run(RequestBatch.of(Request.mechanismRecovery(3, 0)));
        assertEquals(RequestStatus.State.DONE, rig.status(3).state());
        assertEquals("exited", rig.status(2).note());
        assertEquals(RequestStatus.State.DONE, rig.status(2).state());
        assertEquals(0.0, rig.last.motor(FEEDER), 0.0);
        assertEquals("release restores the manual demand", 1.0, rig.last.motor(INTAKE), 0.0);
    }

    @Test
    public void reverseDuringAShotHoldsTheFeedWithoutTimingOutThenFinishes() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(1, 2)));
        rig.runUntilFeeding();
        rig.run(RequestBatch.of(Request.mechanismRecovery(2, 1)));
        assertEquals("the pulse is cut at once", -1.0, rig.last.motor(FEEDER), 0.0);

        // Longer than the 3 s prepare timeout: the hold must not fault the shot.
        for (int i = 0; i < 200; i++) {
            rig.run(RequestBatch.idle());
            assertEquals(-1.0, rig.last.motor(FEEDER), 0.0);
            assertEquals(-1.0, rig.last.motor(INTAKE), 0.0);
            assertTrue("flywheel keeps its target", rig.shooter.targetRpm() > 0.0);
        }
        assertEquals(RequestStatus.State.ACTIVE, rig.status(1).state());
        assertEquals("mechanism recovery", rig.status(1).note());

        rig.run(RequestBatch.of(Request.mechanismRecovery(3, 0)));
        assertDone(rig.runUntilTerminal(1));
    }

    @Test
    public void jamClearCancelsTheShotAndReversesOnlyAfterAMeasuredStop() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(1, 1)));
        rig.run(RequestBatch.idle());
        int jamStart = rig.trace.size();
        rig.run(RequestBatch.of(Request.mechanismRecovery(2, 2)));
        RequestStatus cancelled = rig.status(1);
        assertEquals(RequestStatus.State.REJECTED, cancelled.state());
        assertEquals("jam clear", cancelled.note());
        assertEquals(1.0, rig.last.motor(FLYWHEEL), 0.0);
        assertEquals(1.0, rig.last.motor(FEEDER), 0.0);
        assertEquals(1.0, rig.last.motor(INTAKE), 0.0);
        rig.run(RequestBatch.idle());
        assertEquals(1.0 - PairedHood.servoFraction(RobotConstants.JAM_CLEAR_HOOD_DEG),
                rig.last.servos().get(RobotConstants.HOOD_LEFT_SERVO_NAME), 1e-12);

        for (int i = 0; i < 150; i++) {
            rig.run(RequestBatch.idle());
            assertEquals(1.0, rig.last.motor(INTAKE), 0.0);
        }
        rig.assertNeverDrivenAgainstRotation();

        List<Double> signs = new ArrayList<>();
        long lastRunStart = -1;
        for (Sample sample : rig.trace.subList(jamStart, rig.trace.size())) {
            double sign = Math.signum(sample.flywheel);
            if (sign != 0.0 && (signs.isEmpty() || signs.get(signs.size() - 1) != sign)) {
                signs.add(sign);
                if (lastRunStart >= 0 && signs.size() > 2) {
                    assertTrue("each run lasts the archive toggle time",
                            sample.t - lastRunStart >= RobotConstants.JAM_CLEAR_TOGGLE_MS);
                }
                lastRunStart = sample.t;
            }
            if (sample.flywheel != 0.0) {
                assertEquals("feeder follows the flywheel", sample.flywheel, sample.feeder, 0.0);
            }
        }
        assertTrue("toggles " + signs, signs.size() >= 4);
        assertEquals(1.0, signs.get(0), 0.0);
        for (int i = 1; i < signs.size(); i++) {
            assertEquals(-signs.get(i - 1), signs.get(i), 0.0);
        }
    }

    @Test
    public void shooterRequestsAreRefusedUntilTheExitCoastEnds() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.mechanismRecovery(1, 2)));
        for (int i = 0; i < 10; i++) {
            rig.run(RequestBatch.idle());
        }
        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        assertEquals(RequestStatus.State.REJECTED, rig.status(2).state());
        assertEquals("mechanism recovery owns the flywheel", rig.status(2).note());

        rig.run(RequestBatch.of(Request.mechanismRecovery(3, 0)));
        assertEquals(RequestStatus.State.DONE, rig.status(3).state());
        assertEquals("flywheel coasting", rig.status(1).note());
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
        assertEquals(0.0, rig.last.motor(INTAKE), 0.0);

        rig.run(RequestBatch.of(Request.spinUp(4, 3000.0)));
        assertEquals(RequestStatus.State.REJECTED, rig.status(4).state());

        RequestStatus exited = rig.runUntilTerminal(1);
        assertEquals(RequestStatus.State.DONE, exited.state());
        assertEquals("exited", exited.note());
        assertTrue(rig.shooter.isStopped());

        rig.run(RequestBatch.of(Request.shoot(5, 1)));
        assertDone(rig.runUntilTerminal(5));
        rig.assertNeverDrivenAgainstRotation();
    }

    @Test
    public void cancelAllStopsAJamClearButTheWheelStillBlocksShooting() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.mechanismRecovery(1, 2)));
        for (int i = 0; i < 5; i++) {
            rig.run(RequestBatch.idle());
        }
        rig.run(RequestBatch.cancelAll());
        assertEquals(RequestStatus.State.REJECTED, rig.status(1).state());
        assertEquals("engine switch", rig.status(1).note());
        assertEquals(0.0, rig.last.motor(FLYWHEEL), 0.0);
        assertEquals(0.0, rig.last.motor(FEEDER), 0.0);
        assertEquals(0.0, rig.last.motor(INTAKE), 0.0);

        rig.run(RequestBatch.of(Request.shoot(2, 1)));
        assertEquals(RequestStatus.State.REJECTED, rig.status(2).state());
        for (int i = 0; i < 10 && rig.engine.recovery().active(); i++) {
            rig.run(RequestBatch.idle());
        }
        assertEquals(MechanismRecovery.Phase.OFF, rig.engine.recovery().phase());
        rig.run(RequestBatch.of(Request.shoot(3, 1)));
        assertDone(rig.runUntilTerminal(3));
    }

    @Test
    public void newModeSupersedesAndPerIdCancelStopsReverse() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.intakeOn(1, 0.5)));
        rig.run(RequestBatch.of(Request.mechanismRecovery(2, 1)));
        rig.run(RequestBatch.of(Request.mechanismRecovery(3, 1)));
        assertEquals("superseded", rig.status(2).note());
        assertEquals(RequestStatus.State.DONE, rig.status(2).state());

        rig.run(new RequestBatch(RequestStream.idle(), List.of(), new int[] {3}));
        assertEquals(RequestStatus.State.REJECTED, rig.status(3).state());
        assertEquals(0.0, rig.last.motor(FEEDER), 0.0);
        assertEquals(0.5, rig.last.motor(INTAKE), 0.0);
        assertEquals(MechanismRecovery.Phase.OFF, rig.engine.recovery().phase());
    }

    @Test
    public void malformedModesAreRejected() {
        Rig rig = new Rig();
        Request[] bad = {
                Request.of(1, RequestType.MECHANISM_RECOVERY),
                Request.of(2, RequestType.MECHANISM_RECOVERY, 3.0),
                Request.of(3, RequestType.MECHANISM_RECOVERY, 1.5),
                Request.of(4, RequestType.MECHANISM_RECOVERY, -1.0),
                Request.of(5, RequestType.MECHANISM_RECOVERY, Double.NaN),
        };
        for (Request request : bad) {
            rig.run(RequestBatch.of(request));
            assertEquals(RequestStatus.State.REJECTED, rig.status(request.id()).state());
        }
        assertEquals(MechanismRecovery.Phase.OFF, rig.engine.recovery().phase());
    }

    private static void assertDone(RequestStatus status) {
        assertEquals(status.note(), RequestStatus.State.DONE, status.state());
    }

    // ---- harness -----------------------------------------------------------------------

    private record Sample(long t, double measuredRpm, double flywheel, double feeder,
                          boolean openLoop) { }

    private static final class Rig {
        final FlywheelShooter shooter = new FlywheelShooter();
        final CplxEngine1 engine;
        final List<Sample> trace = new ArrayList<>();
        final Pose pose = new Pose(24.0, 48.0, 0.0);
        long t = -20;
        double wheelRpm;
        RobotAction last;
        List<RequestStatus> statuses = Collections.emptyList();

        Rig() {
            engine = new CplxEngine1(new Subsystems(new FixedDrive(pose), shooter,
                    new PowerIntake(), new LockedTurret()), MechanismProfile.REAL);
            for (int i = 0; i < 20; i++) {
                run(RequestBatch.idle());
            }
        }

        void run(RequestBatch batch) {
            t += 20;
            double measured = wheelRpm;
            Map<String, Double> vel = Map.of(RobotConstants.SHOOTER_FEEDBACK_ENCODER_NAME,
                    measured * TICKS_AT_4000 / 4000.0);
            engine.sense(new RobotState(t, Map.of(), vel, pose.heading(), pose, 12.0));
            engine.act(batch);
            last = engine.action();
            statuses = engine.drainStatuses();
            double power = last.motor(FLYWHEEL);
            trace.add(new Sample(t, measured, power, last.motor(FEEDER), shooter.isOpenLoop()));
            stepPlant(power);
        }

        private void stepPlant(double power) {
            if (shooter.targetRpm() > 0.0 && power != 0.0) {
                // Ideal closed loop, as in FixedShotCoordinatorTest; the guard's zero
                // output falls through to coasting.
                wheelRpm = shooter.targetRpm();
            } else if (power != 0.0) {
                wheelRpm = power * OPEN_LOOP_RPM_AT_FULL_POWER;
            } else {
                wheelRpm = Math.signum(wheelRpm)
                        * Math.max(0.0, Math.abs(wheelRpm) - COAST_STEP_RPM);
            }
        }

        void assertNeverDrivenAgainstRotation() {
            double tolerance = RobotConstants.SHOOTER_TOLERANCE_RPM;
            for (Sample sample : trace) {
                // Closed-loop braking of a forward wheel is archive behavior and allowed.
                boolean against = sample.flywheel * sample.measuredRpm < 0.0
                        && Math.abs(sample.measuredRpm) > tolerance
                        && (sample.openLoop || sample.flywheel > 0.0);
                assertTrue("t=" + sample.t + " power " + sample.flywheel + " at "
                        + sample.measuredRpm + " RPM", !against);
            }
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

        void runUntilFeeding() {
            for (int i = 0; i < 1000; i++) {
                run(RequestBatch.idle());
                if (last.motor(FEEDER) > 0.0) {
                    return;
                }
            }
            throw new AssertionError("never fed: " + statuses);
        }
    }

    private record FixedDrive(Pose pose) implements IDrive {
        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void manual(double vx, double vy, double omega) {}
        @Override public void follow(PathRequest request) {}
        @Override public void stop() {}
        @Override public boolean pathDone() { return true; }
    }

    private static final class LockedTurret implements ITurret {
        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void setRobotPose(Pose pose) {}
        @Override public void aimAt(double fieldX, double fieldY) {}
        @Override public AimResult aimRelative(double angleRad) { return AimResult.ACCEPTED; }
        @Override public AimResult aimStatus() { return AimResult.ACCEPTED; }
        @Override public void scan() {}
        @Override public void hold() {}
        @Override public void disable() {}
        @Override public boolean onTarget() { return true; }
        @Override public double angleRad() { return 0.0; }
    }
}
