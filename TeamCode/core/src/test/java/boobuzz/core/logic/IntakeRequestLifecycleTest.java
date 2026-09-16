package boobuzz.core.logic;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.logic.cplx1.CplxEngine1;
import boobuzz.core.logic.direct.DirectEngine;
import boobuzz.core.subsystem.IDrive;
import boobuzz.core.subsystem.IIntake;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.stub.StubShooter;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Intake actuator ownership must not create a second terminal status. */
public class IntakeRequestLifecycleTest {

    @Test
    public void directIntakeDoneIsNotRejectedWhenItsOwnerIsCancelled() {
        RecordingIntake intake = new RecordingIntake();
        DirectEngine engine = new DirectEngine(
                new Subsystems(new RecordingDrive(), new StubShooter(), intake));
        engine.sense(state(0));
        engine.act(RequestBatch.of(Request.intakeOn(17, 1.0)));
        assertEquals(1, terminalStatuses(engine.drainStatuses(), 17));
        engine.sense(state(20));
        engine.act(new RequestBatch(null, List.of(), new int[] {17}));
        assertEquals(0, terminalStatuses(engine.drainStatuses(), 17));
        assertTrue(intake.stopCalls > 0);
    }

    @Test
    public void cplxIntakeDoneIsNotRejectedWhenItsOwnerIsCancelled() {
        RecordingIntake intake = new RecordingIntake();
        CplxEngine1 engine = new CplxEngine1(
                new Subsystems(new RecordingDrive(), new StubShooter(), intake));
        engine.sense(state(0));
        engine.act(RequestBatch.of(Request.intakeOn(17, 1.0)));
        assertEquals(1, terminalStatuses(engine.drainStatuses(), 17));
        engine.sense(state(20));
        engine.act(new RequestBatch(null, List.of(), new int[] {17}));
        assertEquals(0, terminalStatuses(engine.drainStatuses(), 17));
        assertTrue(intake.stopCalls > 0);
    }

    private static int terminalStatuses(List<RequestStatus> statuses, int id) {
        int count = 0;
        for (RequestStatus status : statuses) {
            if (status.id() == id && status.terminal()) count++;
        }
        return count;
    }

    private static RobotState state(long t) {
        return new RobotState(t, Map.of(), Map.of(), 0.0, Pose.zero(), 12.0);
    }

    private static final class RecordingDrive implements IDrive {
        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void manual(double vx, double vy, double omega) {}
        @Override public void follow(PathRequest request) {}
        @Override public void stop() {}
        @Override public boolean pathDone() { return false; }
        @Override public Pose pose() { return Pose.zero(); }
    }

    private static final class RecordingIntake implements IIntake {
        int stopCalls;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void run(double power) {}
        @Override public void stop() { stopCalls++; }
        @Override public boolean hasBall() { return false; }
    }
}
