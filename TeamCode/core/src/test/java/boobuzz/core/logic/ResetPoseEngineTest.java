package boobuzz.core.logic;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.logic.cplx1.CplxEngine1;
import boobuzz.core.logic.direct.DirectEngine;
import boobuzz.core.subsystem.IDrive;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.stub.StubIntake;
import boobuzz.core.subsystem.stub.StubShooter;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertEquals;

public class ResetPoseEngineTest {

    @Test
    public void directEngineResetsDrivePoseAndCompletesRequest() {
        RecordingDrive drive = new RecordingDrive();
        DirectEngine engine = new DirectEngine(subsystems(drive));
        engine.sense(state(0));
        engine.act(RequestBatch.of(Request.resetPose(7, 24.0, 96.0, Math.PI / 2.0)));

        assertPose(24.0, 96.0, Math.PI / 2.0, drive.resetPose);
        assertEquals(RequestStatus.State.DONE, status(engine, 7).state());
    }

    @Test
    public void cplxEngineResetsDrivePoseAndCompletesRequest() {
        RecordingDrive drive = new RecordingDrive();
        CplxEngine1 engine = new CplxEngine1(subsystems(drive));
        engine.sense(state(0));
        engine.act(RequestBatch.of(Request.resetPose(8, 30.0, 70.0, -0.25)));

        assertPose(30.0, 70.0, -0.25, drive.resetPose);
        assertEquals(RequestStatus.State.DONE, status(engine, 8).state());
    }

    private static Subsystems subsystems(RecordingDrive drive) {
        return new Subsystems(drive, new StubShooter(), new StubIntake());
    }

    private static RequestStatus status(IRobotEngine engine, int id) {
        return engine.drainStatuses().stream()
                .filter(status -> status.id() == id).findFirst().orElseThrow();
    }

    private static void assertPose(double x, double y, double heading, Pose actual) {
        assertEquals(x, actual.x(), 1e-9);
        assertEquals(y, actual.y(), 1e-9);
        double normalized = heading < 0.0 ? heading + 2.0 * Math.PI : heading;
        assertEquals(normalized, actual.heading(), 1e-9);
    }

    private static RobotState state(long t) {
        return new RobotState(t, Map.of(), Map.of(), 0.0,
                new Pose(10.0, 20.0, 0.5), 12.0);
    }

    private static final class RecordingDrive implements IDrive {
        Pose pose = new Pose(0.0, 0.0, 0.0);
        Pose resetPose;

        @Override public void observe(RobotState state) { pose = state.pinpoint(); }
        @Override public void update(RobotAction.Builder out) {}
        @Override public void manual(double vx, double vy, double omega) {}
        @Override public void follow(PathRequest request) {}
        @Override public void stop() {}
        @Override public void resetPose(Pose pose) {
            this.resetPose = pose;
            this.pose = pose;
        }
        @Override public boolean pathDone() { return true; }
        @Override public Pose pose() { return pose; }
    }
}
