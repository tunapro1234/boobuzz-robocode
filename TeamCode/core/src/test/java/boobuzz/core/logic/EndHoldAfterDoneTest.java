package boobuzz.core.logic;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RequestType;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.logic.cplx1.CplxEngine1;
import boobuzz.core.logic.direct.DirectEngine;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.pedro.PedroDrive;
import boobuzz.core.subsystem.stub.StubIntake;
import boobuzz.core.subsystem.stub.StubShooter;

import com.pedropathing.math.Pose;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;

import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * After a drive request reports DONE the follower keeps holding the end pose, as the
 * archive and simple-code do, until a new drive request, manual drive, a cancel of
 * that id, CANCEL_ALL or RESET_POSE.
 */
@RunWith(Parameterized.class)
public class EndHoldAfterDoneTest {

    private static final long DT_MS = 20L;
    private static final int GOTO_ID = 5;
    private static final Pose TARGET = new Pose(24.0, 0.0, 0.0);
    /** Pushed 4 in back along the path after DONE. */
    private static final Pose PUSHED = new Pose(20.0, 0.0, 0.0);
    private static final PathRequest SAME_PATH =
            PathRequest.goTo(TARGET, PathRequest.Constraints.defaults());
    private static final String[] WHEELS = {"fl", "fr", "bl", "br"};

    @Parameterized.Parameters(name = "{0}")
    public static List<Object[]> engines() {
        Supplier<IRobotEngine> direct = () -> new DirectEngine(subsystems());
        Supplier<IRobotEngine> cplx1 = () -> new CplxEngine1(subsystems());
        return List.of(new Object[] {"direct", direct}, new Object[] {"cplx1", cplx1});
    }

    private final Supplier<IRobotEngine> factory;
    private IRobotEngine engine;
    private long t;

    public EndHoldAfterDoneTest(String name, Supplier<IRobotEngine> factory) {
        this.factory = factory;
    }

    @Test
    public void holdKeepsCorrectingAPushedRobotAfterDone() {
        driveToDone();
        for (int i = 0; i < 10; i++) {
            RobotAction action = tick(PUSHED, RequestBatch.idle());
            for (String wheel : WHEELS) {
                // Forward correction back to x=24 on every wheel; never zeroed.
                assertTrue(wheel + " not correcting at t=" + t + ": " + action,
                        action.motor(wheel) > 0.0);
            }
        }
    }

    @Test
    public void cancelOfTheDoneIdReleasesTheHold() {
        driveToDone();
        tick(PUSHED, new RequestBatch(RequestStream.idle(), List.of(), new int[] {GOTO_ID}));
        assertZero(tick(PUSHED, RequestBatch.idle()));
    }

    @Test
    public void cancelOfAnotherIdKeepsTheHold() {
        driveToDone();
        tick(PUSHED, new RequestBatch(RequestStream.idle(), List.of(), new int[] {GOTO_ID + 1}));
        assertTrue(tick(PUSHED, RequestBatch.idle()).motor("fl") > 0.0);
    }

    @Test
    public void cancelAllReleasesTheHold() {
        driveToDone();
        tick(PUSHED, RequestBatch.cancelAll());
        assertZero(tick(PUSHED, RequestBatch.idle()));
    }

    @Test
    public void resetPoseReleasesTheHold() {
        driveToDone();
        tick(PUSHED, RequestBatch.of(Request.resetPose(9, 20.0, 0.0, 0.0)));
        assertZero(tick(PUSHED, RequestBatch.idle()));
    }

    @Test
    public void manualDriveReleasesTheHold() {
        driveToDone();
        tick(PUSHED, new RequestBatch(RequestStream.manual(0.0, 0.0, 0.0), List.of()));
        assertZero(tick(PUSHED, RequestBatch.idle()));
    }

    @Test
    public void repeatedPathAfterDoneRestartsThePath() {
        // The same PathRequest instance again, from the pushed pose: a new path, not
        // the finished request's hold (which would report DONE at once).
        driveToDone(Request.path(GOTO_ID, SAME_PATH));
        tick(PUSHED, RequestBatch.of(Request.path(GOTO_ID + 1, SAME_PATH)));
        for (int i = 0; i < 3; i++) {
            tick(PUSHED, RequestBatch.idle());
            assertEquals("pushed 4 in short must not be DONE",
                    RequestStatus.State.ACTIVE, last(GOTO_ID + 1).state());
        }
    }

    private void driveToDone() {
        driveToDone(Request.of(GOTO_ID, RequestType.GOTO,
                TARGET.x(), TARGET.y(), TARGET.heading()));
    }

    private void driveToDone(Request request) {
        engine = factory.get();
        t = 0L;
        tick(new Pose(0.0, 0.0, 0.0), RequestBatch.idle());
        tick(new Pose(0.0, 0.0, 0.0), RequestBatch.of(request));
        for (int i = 0; i < 50; i++) {
            tick(TARGET, RequestBatch.idle());
            if (lastState == RequestStatus.State.DONE) {
                return;
            }
        }
        throw new AssertionError("GOTO never reported DONE");
    }

    private RequestStatus.State lastState;
    private List<RequestStatus> lastStatuses = List.of();

    private RobotAction tick(Pose pose, RequestBatch batch) {
        t += DT_MS;
        engine.sense(new RobotState(t, Map.of(), Map.of(), pose.heading(), pose, 12.0));
        engine.act(batch);
        lastStatuses = engine.drainStatuses();
        for (RequestStatus status : lastStatuses) {
            if (status.id() == GOTO_ID) {
                lastState = status.state();
            }
        }
        return engine.action();
    }

    private RequestStatus last(int id) {
        RequestStatus found = null;
        for (RequestStatus status : lastStatuses) {
            if (status.id() == id) {
                found = status;
            }
        }
        if (found == null) {
            throw new AssertionError("no status for " + id + " at t=" + t);
        }
        return found;
    }

    private static void assertZero(RobotAction action) {
        for (String wheel : WHEELS) {
            assertEquals(wheel + " " + action, 0.0, action.motor(wheel), 0.0);
        }
    }

    private static Subsystems subsystems() {
        return new Subsystems(new PedroDrive(Mechanism.DEFAULT), new StubShooter(),
                new StubIntake());
    }
}
