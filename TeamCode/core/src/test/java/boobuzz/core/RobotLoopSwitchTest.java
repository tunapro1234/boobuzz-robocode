package boobuzz.core;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.GamepadState;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.controller.IController;
import boobuzz.core.hal.IHal;
import boobuzz.core.logic.IRobotEngine;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

public class RobotLoopSwitchTest {

    private static final class FakeHal implements IHal {
        long now;

        @Override public long now() { return now; }

        @Override public RobotState read() {
            return new RobotState(now, Map.of(), Map.of(), 0.0,
                    new Pose(0.0, 0.0, 0.0), 12.0);
        }

        @Override public void write(RobotAction action) { now += 20; }

        @Override public GamepadState get() { return GamepadState.neutral(); }
    }

    private static final class RecordingEngine implements IRobotEngine {
        private final String name;
        private final List<RequestBatch> batches = new ArrayList<>();
        private RobotAction action = RobotAction.zero();

        private RecordingEngine(String name) { this.name = name; }

        @Override public String name() { return name; }

        @Override public WorldSnapshot sense(RobotState state) {
            return new WorldSnapshot(state.t(), state.pinpoint(), state.yaw(), state.voltage());
        }

        @Override public void act(RequestBatch batch) {
            batches.add(batch);
            if (batch.cancels().length == 0) {
                action = RobotAction.ofMotors(Map.of("fl", name.equals("old") ? 0.7 : 0.4));
            }
        }

        @Override public RobotAction action() { return action; }
    }

    @Test
    public void switchCancelsOldEngineAndStartsNewOnNextTick() {
        FakeHal hal = new FakeHal();
        RecordingEngine oldEngine = new RecordingEngine("old");
        RecordingEngine newEngine = new RecordingEngine("new");
        IController controller = new IController() {
            private boolean sent;

            @Override public RequestBatch decide(Feedback feedback) {
                if (sent) return RequestBatch.idle();
                sent = true;
                return RequestBatch.of(Request.switchEngine(7, 1));
            }
        };
        RobotLoop loop = new RobotLoop(hal, List.of(oldEngine, newEngine), oldEngine, controller);

        loop.tick();
        assertSame(newEngine, loop.engine());
        assertEquals(1, oldEngine.batches.size());
        assertTrue(oldEngine.batches.get(0).cancels()[0] == RequestBatch.CANCEL_ALL);
        assertEquals(0, newEngine.batches.size());

        loop.tick();
        assertEquals(1, newEngine.batches.size());
        assertTrue(newEngine.batches.get(0).requests().isEmpty());
    }

    @Test
    public void switchingBackWritesZeroInsteadOfRetainedAction() {
        FakeHal hal = new FakeHal();
        List<RobotAction> writes = new ArrayList<>();
        IHal recordingHal = new IHal() {
            @Override public long now() { return hal.now(); }
            @Override public RobotState read() { return hal.read(); }
            @Override public void write(RobotAction action) {
                writes.add(action);
                hal.write(action);
            }
            @Override public GamepadState get() { return hal.get(); }
        };
        RecordingEngine first = new RecordingEngine("old");
        RecordingEngine second = new RecordingEngine("new");
        IController controller = new IController() {
            private int tick;

            @Override public RequestBatch decide(Feedback feedback) {
                return switch (tick++) {
                    case 0 -> RequestBatch.idle();
                    case 1 -> RequestBatch.of(Request.switchEngine(8, 1));
                    case 2 -> RequestBatch.of(Request.switchEngine(9, 0));
                    default -> RequestBatch.idle();
                };
            }
        };
        RobotLoop loop = new RobotLoop(recordingHal, List.of(first, second), first, controller);

        loop.tick();
        loop.tick();
        loop.tick();

        assertEquals(0.7, writes.get(0).motor("fl"), 1e-9);
        assertEquals(0.0, writes.get(1).motor("fl"), 1e-9);
        assertEquals(0.0, writes.get(2).motor("fl"), 1e-9);
    }
}
