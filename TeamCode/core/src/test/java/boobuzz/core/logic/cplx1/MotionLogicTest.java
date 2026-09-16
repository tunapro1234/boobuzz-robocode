package boobuzz.core.logic.cplx1;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.subsystem.IDrive;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;

public class MotionLogicTest {

    @Test
    public void manualStreamWinsAndPathCompletes() {
        RecordingDrive drive = new RecordingDrive();
        MotionLogic logic = new MotionLogic(drive);
        List<RequestStatus> statuses = new ArrayList<>();

        logic.act(RequestStream.idle(),
                List.of(Request.path(7, PathRequest.named("test-line"))), new int[0], statuses);
        assertEquals(RequestStatus.State.ACTIVE, statuses.get(0).state());

        statuses.clear();
        logic.act(RequestStream.manual(0.2, -0.3, 0.4), List.of(), new int[0], statuses);
        assertEquals(RequestStatus.State.REJECTED, statuses.get(0).state());
        assertEquals(0.2, drive.vx, 1e-9);

        statuses.clear();
        drive.pathDone = true;
        logic.act(RequestStream.idle(),
                List.of(Request.of(9, boobuzz.core.contract.RequestType.TURN_TO, 1.0)),
                new int[0], statuses);
        assertEquals(RequestStatus.State.ACTIVE, statuses.get(0).state());
    }

    private static final class RecordingDrive implements IDrive {
        double vx;
        boolean pathDone;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void manual(double vx, double vy, double omega) { this.vx = vx; }
        @Override public void follow(PathRequest request) { pathDone = false; }
        @Override public void turnTo(double headingRad) { pathDone = false; }
        @Override public void stop() {}
        @Override public boolean pathDone() { return pathDone; }
        @Override public Pose pose() { return Pose.zero(); }
    }
}
