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

        private RecordingEngine(String name) { this.name = name; }

        @Override public String name() { return name; }

        @Override public WorldSnapshot sense(RobotState state) {
            return new WorldSnapshot(state.t(), state.pinpoint(), state.yaw(), state.voltage());
        }

        @Override public void act(RequestBatch batch) { batches.add(batch); }
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
}
