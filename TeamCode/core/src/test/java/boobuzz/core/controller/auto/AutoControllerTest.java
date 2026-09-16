package boobuzz.core.controller.auto;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestType;
import boobuzz.core.contract.WorldSnapshot;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class AutoControllerTest {

    @Test
    public void pathEmitsAttachedRequestsAndStopsIntakeAfterDone() {
        AutoSequence sequence = AutoBuilder.start(new Pose(0.0, 0.0, 0.0))
                .lineTo(10.0, 0.0)
                .withIntake()
                .withShooterWarmup(300.0)
                .build();
        AutoController controller = new AutoController(sequence);

        RequestBatch first = controller.decide(feedback(0));
        assertEquals(3, first.requests().size());
        assertEquals(RequestType.PATH, first.requests().get(0).type());
        assertEquals(RequestType.INTAKE_ON, first.requests().get(1).type());
        assertEquals(RequestType.SPIN_UP, first.requests().get(2).type());

        int pathId = first.requests().get(0).id();
        RequestBatch active = controller.decide(feedback(20,
                active(pathId), active(first.requests().get(1).id()),
                active(first.requests().get(2).id())));
        assertTrue(active.requests().isEmpty());

        RequestBatch stopIntake = controller.decide(feedback(40, RequestStatus.done(pathId)));
        assertEquals(RequestType.INTAKE_OFF, stopIntake.requests().get(0).type());
        int offId = stopIntake.requests().get(0).id();

        controller.decide(feedback(60, RequestStatus.done(offId)));
        assertTrue(controller.isDone());
    }

    @Test
    public void waitUsesTheFeedbackClock() {
        AutoController controller = new AutoController(
                AutoBuilder.start(new Pose(0.0, 0.0, 0.0)).waitSeconds(0.5).build());

        controller.decide(feedback(100));
        assertFalse(controller.isDone());
        controller.decide(feedback(599));
        assertFalse(controller.isDone());
        controller.decide(feedback(600));
        assertTrue(controller.isDone());
    }

    @Test
    public void modifiersChangeTheMostRecentPathOnly() {
        AutoSequence sequence = AutoBuilder.start(new Pose(0.0, 0.0, 0.0))
                .lineTo(10.0, 0.0)
                .withConstantHeading(90.0)
                .waitSeconds(0.1)
                .lineTo(20.0, 0.0)
                .build();

        AutoStep.Path first = (AutoStep.Path) sequence.steps().get(0);
        AutoStep.Path second = (AutoStep.Path) sequence.steps().get(2);
        assertEquals(PathRequestHeading.CONSTANT, PathRequestHeading.of(first));
        assertEquals(PathRequestHeading.TANGENT, PathRequestHeading.of(second));
        assertEquals(90.0, Math.toDegrees(first.request().heading().start()), 1e-9);
    }

    @Test
    public void rejectionStopsSequenceAndExposesFailure() {
        AutoController controller = new AutoController(
                new AutoSequence(List.of(new AutoStep.Turn(0.5))));
        RequestBatch first = controller.decide(feedback(0));
        RequestStatus rejected = RequestStatus.rejected(
                first.requests().get(0).id(), "unsupported");

        RequestBatch after = controller.decide(feedback(20, rejected));
        assertTrue(after.requests().isEmpty());
        assertTrue(controller.isFailed());
        assertNotNull(controller.failure());
        assertEquals("unsupported", controller.failureNote());
        assertTrue(controller.isFinished());
    }

    @Test
    public void builderCompilesCurveAndLinearHeading() {
        AutoSequence sequence = AutoBuilder.start(new Pose(0.0, 0.0, 0.0))
                .curveTo(new Pose(20.0, 10.0), new Pose(8.0, 0.0), new Pose(12.0, 10.0))
                .withLinearHeading(0.0, 45.0)
                .build();
        AutoStep.Path path = (AutoStep.Path) sequence.current();
        assertEquals(1, path.request().segments().size());
        assertEquals(2, ((boobuzz.core.contract.PathRequest.Curve)
                path.request().segments().get(0)).controlPoints().size());
        assertEquals(45.0, Math.toDegrees(path.request().heading().end()), 1e-9);
    }

    private static RequestStatus active(int id) {
        return new RequestStatus(id, RequestStatus.State.ACTIVE, 0.0, "active");
    }

    private static Feedback feedback(long t, RequestStatus... statuses) {
        return new Feedback(new WorldSnapshot(t, new Pose(0.0, 0.0, 0.0),
                0.0, 12.0), List.of(statuses), t);
    }

    private enum PathRequestHeading {
        TANGENT, CONSTANT;

        static PathRequestHeading of(AutoStep.Path path) {
            return path.request().heading().mode()
                    == boobuzz.core.contract.PathRequest.HeadingMode.CONSTANT
                    ? CONSTANT : TANGENT;
        }
    }
}
