package boobuzz.core;

import boobuzz.core.contract.GamepadState;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.controller.IController;
import boobuzz.core.hal.IHal;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.logic.direct.DirectEngine;
import boobuzz.core.subsystem.IDrive;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.stub.StubIntake;
import boobuzz.core.subsystem.stub.StubShooter;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Direct-engine, HAL-cadence, and loop-shutdown coverage for the R3 safety paths. */
public class R3SafetyCoverageTest {

    @Test
    public void directEnginePathIsStoppedByManualTakeover() {
        RecordingDrive drive = new RecordingDrive();
        DirectEngine engine = new DirectEngine(
                new Subsystems(drive, new StubShooter(), new StubIntake()));
        TestHal hal = new TestHal();
        IController controller = new IController() {
            private int tick;

            @Override public RequestBatch decide(boobuzz.core.contract.Feedback feedback) {
                return tick++ == 0
                        ? RequestBatch.of(Request.path(41, PathRequest.named("test-line")))
                        : new RequestBatch(RequestStream.manual(0.25, 0.0, 0.0), List.of());
            }
        };
        try (RobotLoop loop = new RobotLoop(hal, engine, controller)) {
            loop.tick();
            loop.tick();
            assertTrue(drive.stopCalls > 0);
            assertEquals(0.25, hal.last.motor("fl"), 1e-9);
        }
    }

    @Test
    public void sdkShapedHalMaintainsOneThousandTickCadence() {
        TestHal hal = new TestHal();
        try (RobotLoop loop = new RobotLoop(hal,
                new DirectEngine(new Subsystems(new RecordingDrive(),
                        new StubShooter(), new StubIntake())),
                feedback -> new RequestBatch(RequestStream.manual(0.2, 0.0, 0.0), List.of()))) {
            for (int i = 0; i < 1000; i++) loop.tick();
            assertEquals(1000, loop.ticks());
            assertEquals(20_000L, hal.now());
            assertTrue(Double.isFinite(loop.meanTickMillis()));
            assertTrue(Double.isFinite(loop.maxTickMillis()));
        }
    }

    @Test
    public void loopShutdownFlushesBagWithoutLeavingDispatcher() throws Exception {
        Path path = Files.createTempFile("r3-shutdown", ".jsonl");
        TestHal hal = new TestHal();
        RobotLoop loop = new RobotLoop(hal,
                new DirectEngine(new Subsystems(new RecordingDrive(),
                        new StubShooter(), new StubIntake())),
                feedback -> RequestBatch.idle());
        try {
            assertTrue(loop.openBag(path.toFile(), "R3SafetyCoverageTest"));
            loop.tick();
        } finally {
            loop.close();
        }
        List<String> lines = Files.readAllLines(path);
        assertTrue(lines.size() >= 4);
        assertTrue(lines.get(0).contains("\"bag\":1"));
        Files.deleteIfExists(path);
    }

    private static final class TestHal implements IHal {
        private long now;
        private RobotAction last = RobotAction.zero();

        @Override public long now() { return now; }

        @Override public RobotState read() {
            return new RobotState(now, Map.of(), Map.of(), 0.0,
                    Pose.zero(), 12.6);
        }

        @Override public void write(RobotAction action) {
            last = action;
            now += 20;
        }

        @Override public GamepadState get() { return GamepadState.neutral(); }
    }

    private static final class RecordingDrive implements IDrive {
        int stopCalls;
        private boolean following;
        private double manualPower;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {
            out.motor("fl", following ? 1.0 : manualPower);
        }
        @Override public void manual(double vx, double vy, double omega) {
            following = false;
            manualPower = vx;
        }
        @Override public void follow(PathRequest request) { following = true; }
        @Override public void stop() { stopCalls++; following = false; manualPower = 0.0; }
        @Override public boolean pathDone() { return false; }
        @Override public Pose pose() { return Pose.zero(); }
    }
}
