package boobuzz.core.controller.teleop;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.GamepadState;
import boobuzz.core.contract.IGamepadSource;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestType;
import boobuzz.core.contract.WorldSnapshot;

import com.pedropathing.math.Pose;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Field-oriented drive conversion ends in the robot-frame request stream. */
public class TeleopControllerTest {

    private static final double EPS = 1e-9;

    private static final class Pad implements IGamepadSource {
        GamepadState state = GamepadState.neutral();

        @Override public GamepadState get() { return state; }
    }

    private static GamepadState forwardStick() {
        return new GamepadState(0, -1, 0, 0,
                false, false, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
    }

    private static Feedback at(double heading) {
        return Feedback.of(new WorldSnapshot(0, new Pose(72, 72, heading), heading, 12.6));
    }

    private static RequestStream streamOf(boobuzz.core.contract.RequestBatch batch) {
        assertTrue("expected manual stream", batch.stream().manualDrive());
        return batch.stream();
    }

    @Test
    public void fieldOrientedIsOnByDefault() {
        Pad pad = new Pad();
        pad.state = forwardStick();
        RequestStream d = streamOf(new TeleopController(pad).decide(at(Math.PI / 2)));
        assertEquals(0.0, d.vx(), EPS);
        assertEquals(-1.0, d.vy(), EPS);
    }

    @Test
    public void forwardStickUnchangedAtHeadingZero() {
        Pad pad = new Pad();
        pad.state = forwardStick();
        RequestStream d = streamOf(new TeleopController(pad).decide(at(0)));
        assertEquals(1.0, d.vx(), EPS);
        assertEquals(0.0, d.vy(), EPS);
    }

    @Test
    public void forwardStickRotatesToVyAtHalfPiHeading() {
        Pad pad = new Pad();
        pad.state = forwardStick();
        RequestStream d = streamOf(new TeleopController(pad).decide(at(Math.PI / 2)));
        assertEquals(0.0, d.vx(), EPS);
        assertEquals(-1.0, d.vy(), EPS);
    }

    @Test
    public void headingIgnoredInRobotOrientedMode() {
        Pad pad = new Pad();
        TeleopController controller = new TeleopController(pad);
        pad.state = new GamepadState(0, -1, 0, 0,
                false, true, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
        RequestStream d = streamOf(controller.decide(at(Math.PI / 2)));
        assertEquals(1.0, d.vx(), EPS);
        assertEquals(0.0, d.vy(), EPS);
    }

    @Test
    public void toggleTriggersOnEdgeNotWhileHeld() {
        Pad pad = new Pad();
        TeleopController controller = new TeleopController(pad);
        GamepadState bHeld = new GamepadState(0, -1, 0, 0,
                false, true, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
        pad.state = bHeld;
        assertEquals(1.0, streamOf(controller.decide(at(Math.PI / 2))).vx(), EPS);
        assertEquals(1.0, streamOf(controller.decide(at(Math.PI / 2))).vx(), EPS);
        assertEquals(1.0, streamOf(controller.decide(at(Math.PI / 2))).vx(), EPS);
        pad.state = GamepadState.neutral();
        controller.decide(at(0));
        pad.state = bHeld;
        RequestStream toggledBack = streamOf(controller.decide(at(Math.PI / 2)));
        assertEquals(0.0, toggledBack.vx(), EPS);
        assertEquals(-1.0, toggledBack.vy(), EPS);
    }

    @Test
    public void yResetMakesCurrentHeadingForward() {
        Pad pad = new Pad();
        TeleopController controller = new TeleopController(pad);
        pad.state = new GamepadState(0, -1, 0, 0,
                false, false, false, true, false, false, 0, 0, GamepadState.Dpad.NONE);
        RequestStream d = streamOf(controller.decide(at(Math.PI / 2)));
        assertEquals(1.0, d.vx(), EPS);
        assertEquals(0.0, d.vy(), EPS);
    }

    @Test
    public void rotationIsRobotFrameInBothModes() {
        Pad pad = new Pad();
        pad.state = new GamepadState(0, 0, -1, 0,
                false, false, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
        assertEquals(1.0, streamOf(new TeleopController(pad)
                .decide(at(Math.PI / 3))).omega(), EPS);
    }

    @Test
    public void yawUsedWhenPoseMissing() {
        Pad pad = new Pad();
        pad.state = forwardStick();
        Feedback noPose = Feedback.of(new WorldSnapshot(0, null, Math.PI / 2, 12.6));
        RequestStream d = streamOf(new TeleopController(pad).decide(noPose));
        assertEquals(0.0, d.vx(), EPS);
        assertEquals(-1.0, d.vy(), EPS);
    }

    @Test
    public void subDeadbandStickDoesNotCancelPath() {
        Pad pad = new Pad();
        TeleopController controller = new TeleopController(pad);
        pad.state = new GamepadState(0, 0, 0, 0,
                false, false, false, true, false, false, 0, 0,
                GamepadState.Dpad.NONE);
        RequestBatch sequence = controller.decide(at(0.0));
        assertTrue(sequence.requests().stream()
                .anyMatch(request -> request.type() == RequestType.PATH));

        pad.state = new GamepadState(0, -0.01, 0, 0,
                false, false, false, false, false, false, 0, 0,
                GamepadState.Dpad.NONE);
        RequestBatch quiet = controller.decide(new Feedback(
                new WorldSnapshot(20, new Pose(72, 72, 0.0), 0.0, 12.6),
                java.util.List.of(), 20));

        assertFalse(quiet.stream().manualDrive());
        assertEquals(0, quiet.cancels().length);
        assertTrue(controller.sequenceRunner() != null
                && !controller.sequenceRunner().isFailed());
    }

    @Test
    public void manualTakeoverCancelsTheActiveSequenceRequest() {
        Pad pad = new Pad();
        TeleopController controller = new TeleopController(pad);
        pad.state = new GamepadState(0, 0, 0, 0,
                false, false, false, true, false, false, 0, 0,
                GamepadState.Dpad.NONE);
        RequestBatch sequence = controller.decide(at(0.0));
        int pathId = sequence.requests().stream()
                .filter(request -> request.type() == RequestType.PATH)
                .findFirst().orElseThrow().id();

        pad.state = forwardStick();
        RequestBatch manual = controller.decide(new Feedback(
                new WorldSnapshot(20, new Pose(72, 72, 0.0), 0.0, 12.6),
                java.util.List.of(), 20));

        assertTrue(manual.stream().manualDrive());
        assertTrue(java.util.Arrays.stream(manual.cancels()).anyMatch(id -> id == pathId));
        assertTrue(controller.sequenceRunner().isFailed());
    }
}
