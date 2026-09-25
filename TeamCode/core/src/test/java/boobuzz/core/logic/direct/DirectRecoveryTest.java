package boobuzz.core.logic.direct;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.IDrive;
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
 * MECHANISM_RECOVERY through the direct engine over the real FlywheelShooter and
 * PowerIntake. Plant as in the cplx1 MechanismRecoveryEngineTest (test fixtures).
 */
public class DirectRecoveryTest {

    private static final String FEEDER = RobotConstants.FEEDER_MOTOR_NAME;
    private static final String INTAKE = RobotConstants.INTAKE_MOTOR_NAME;
    private static final String FLYWHEEL = RobotConstants.SHOOTER_RIGHT_MOTOR_NAME;
    private static final double TICKS_AT_4000 = 1166.6666667;
    /** Test fixture, not a measured coast rate. */
    private static final double COAST_STEP_RPM = 1000.0;
    /** Test fixture: open-loop speed at full power. */
    private static final double OPEN_LOOP_RPM_AT_FULL_POWER = 4000.0;

    @Test
    public void reverseHoldsTheShotFeedAndRestoresManualIntake() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.intakeOn(1, 0.6)));
        rig.run(RequestBatch.of(Request.mechanismRecovery(2, 1)));
        rig.run(RequestBatch.of(Request.shoot(3, 2, 4000.0)));
        for (int i = 0; i < 50; i++) {
            rig.run(RequestBatch.idle());
            assertEquals(-1.0, rig.last.motor(FEEDER), 0.0);
            assertEquals(-1.0, rig.last.motor(INTAKE), 0.0);
        }
        assertEquals("mechanism recovery", rig.status(3).note());

        rig.run(RequestBatch.of(Request.mechanismRecovery(4, 0)));
        assertEquals(RequestStatus.State.DONE, rig.status(2).state());
        assertEquals(0.6, rig.last.motor(INTAKE), 0.0);
        RequestStatus shot = rig.runUntilTerminal(3);
        assertEquals(shot.note(), RequestStatus.State.DONE, shot.state());
    }

    @Test
    public void jamClearCancelsTheShotAndBlocksShootingUntilStopped() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.shoot(1, 1, 4000.0)));
        rig.run(RequestBatch.of(Request.mechanismRecovery(2, 2)));
        assertEquals(RequestStatus.State.REJECTED, rig.status(1).state());
        assertEquals("jam clear", rig.status(1).note());
        assertEquals(1.0, rig.last.motor(FLYWHEEL), 0.0);
        assertEquals(1.0, rig.last.motor(INTAKE), 0.0);

        boolean reversed = false;
        for (int i = 0; i < 60; i++) {
            rig.run(RequestBatch.idle());
            reversed |= rig.last.motor(FLYWHEEL) < 0.0;
        }
        assertTrue("jam clear reached the reverse run", reversed);
        rig.run(RequestBatch.of(Request.spinUp(3, 3000.0)));
        assertEquals("mechanism recovery owns the flywheel", rig.status(3).note());

        rig.run(RequestBatch.of(Request.mechanismRecovery(4, 0)));
        RequestStatus exited = rig.runUntilTerminal(2);
        assertEquals("exited", exited.note());
        assertEquals(0.0, rig.last.motor(INTAKE), 0.0);
        rig.run(RequestBatch.of(Request.shoot(5, 1, 4000.0)));
        RequestStatus shot = rig.runUntilTerminal(5);
        assertEquals(shot.note(), RequestStatus.State.DONE, shot.state());
    }

    @Test
    public void cancelAllEndsRecoveryAndReleasesTheIntake() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.intakeOn(1, 1.0)));
        rig.run(RequestBatch.of(Request.mechanismRecovery(2, 1)));
        rig.run(RequestBatch.cancelAll());
        assertEquals("engine switch", rig.status(2).note());
        assertEquals(0.0, rig.last.motor(FEEDER), 0.0);
        assertEquals(0.0, rig.last.motor(INTAKE), 0.0);
        rig.run(RequestBatch.idle());
        assertEquals(0.0, rig.last.motor(INTAKE), 0.0);
    }

    @Test
    public void perIdCancelRestoresManualIntake() {
        Rig rig = new Rig();
        rig.run(RequestBatch.of(Request.intakeOn(1, 0.4)));
        rig.run(RequestBatch.of(Request.mechanismRecovery(2, 1)));
        rig.run(new RequestBatch(RequestStream.idle(), List.of(), new int[] {2}));
        assertEquals("cancelled", rig.status(2).note());
        assertEquals(0.0, rig.last.motor(FEEDER), 0.0);
        assertEquals(0.4, rig.last.motor(INTAKE), 0.0);
    }

    private static final class Rig {
        final FlywheelShooter shooter = new FlywheelShooter();
        final DirectEngine engine;
        final Pose pose = new Pose(24.0, 48.0, 0.0);
        long t = -20;
        double wheelRpm;
        RobotAction last;
        List<RequestStatus> statuses = Collections.emptyList();

        Rig() {
            engine = new DirectEngine(new Subsystems(new FixedDrive(pose), shooter,
                    new PowerIntake()));
            run(RequestBatch.idle());
        }

        void run(RequestBatch batch) {
            t += 20;
            Map<String, Double> vel = Map.of(RobotConstants.SHOOTER_FEEDBACK_ENCODER_NAME,
                    wheelRpm * TICKS_AT_4000 / 4000.0);
            engine.sense(new RobotState(t, Map.of(), vel, pose.heading(), pose, 12.0));
            engine.act(batch);
            last = engine.action();
            statuses = engine.drainStatuses();
            double power = last.motor(FLYWHEEL);
            if (shooter.targetRpm() > 0.0 && power != 0.0) {
                wheelRpm = shooter.targetRpm();
            } else if (power != 0.0) {
                wheelRpm = power * OPEN_LOOP_RPM_AT_FULL_POWER;
            } else {
                wheelRpm = Math.signum(wheelRpm)
                        * Math.max(0.0, Math.abs(wheelRpm) - COAST_STEP_RPM);
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
