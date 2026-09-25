package boobuzz.sim;

import boobuzz.core.RobotFactory;
import boobuzz.core.RobotLoop;
import boobuzz.core.controller.IController;
import boobuzz.core.controller.replay.ReplayController;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.hal.Mechanism;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/** Real Python-server record/replay proof for the deterministic seam contract. */
public class ReplayIntegrationTest {

    private static final int RECORD_TICKS = 10_000;

    @Test
    public void seededRecordAndReplayAreBitEqual() throws Exception {
        File simulator = simulatorRoot();
        File python = pythonExecutable(simulator);
        File root = repositoryRoot();
        File mechanism = new File(root,
                "TeamCode/core/src/main/java/boobuzz/core/hal/RobotConstants.java");
        if (!mechanism.isFile()) {
            throw new AssertionError("RobotConstants.java is missing: " + mechanism);
        }

        File bag = File.createTempFile("robot-replay", ".jsonl");
        // Each server binds port 0 and reports its port; nothing is probed first.
        ChildProcess recordServer = ChildProcess.startSimServer(simulator, python, mechanism);
        int recordPort = recordServer.awaitPort(0L);
        List<PoseBits> recordedTruth = new ArrayList<>();
        try {
            Pose start = new Pose(72.0, 72.0, 0.0);
            Mechanism m = Mechanism.DEFAULT;
            try (SimHal hal = connect(m, recordPort, start);
                 RobotLoop loop = RobotFactory.createWithController(
                         hal, m, "cplx1", new TestLineController(), 0, true)) {
                assertTrueOpenBag(loop, bag, start);
                for (int i = 0; i < RECORD_TICKS; i++) {
                    loop.tick();
                    recordedTruth.add(PoseBits.of(hal.truth()));
                }
            }
        } finally {
            recordServer.close();
        }

        assertTruncatedBagRejected();

        ChildProcess replayServer = ChildProcess.startSimServer(simulator, python, mechanism);
        int replayPort = replayServer.awaitPort(0L);
        try {
            ReplayController replay = new ReplayController(bag);
            assertEquals(RECORD_TICKS, replay.tickCount());
            assertNotNull(replay.initialPose());
            List<PoseBits> replayedTruth = readTruthFromBagRun(replay, m(), replayPort);
            assertEquals("truth sequence length", recordedTruth.size(), replayedTruth.size());
            for (int i = 0; i < recordedTruth.size(); i++) {
                assertEquals("truth tick " + i, recordedTruth.get(i), replayedTruth.get(i));
            }
        } finally {
            replayServer.close();
            if (!bag.delete()) bag.deleteOnExit();
        }
    }

    private static File repositoryRoot() throws IOException {
        File root = new File(System.getProperty("user.dir")).getAbsoluteFile();
        if ("sim".equals(root.getName())) root = root.getParentFile();
        return root.getCanonicalFile();
    }

    private static File simulatorRoot() throws IOException {
        String configured = System.getProperty("ftc.sim.root");
        if (configured == null || configured.trim().isEmpty()) {
            configured = System.getenv("FTC_SIM_ROOT");
        }
        File root;
        if (configured != null && !configured.trim().isEmpty()) {
            root = new File(configured.trim());
        } else {
            root = new File(repositoryRoot().getParentFile(), "re-cock-nize");
        }
        root = root.getCanonicalFile();
        if (!root.isDirectory()) {
            throw new AssertionError("simulator root is missing: " + root
                    + "; set FTC_SIM_ROOT or -Dftc.sim.root");
        }
        return root;
    }

    private static File pythonExecutable(File simulator) {
        String configured = System.getProperty("ftc.sim.python");
        if (configured == null || configured.trim().isEmpty()) {
            configured = System.getenv("PYTHON");
        }
        File python = configured == null || configured.trim().isEmpty()
                ? new File(simulator, ".venv/bin/python")
                : new File(configured.trim());
        if (!python.isFile()) {
            throw new AssertionError("simulator Python executable is missing: " + python
                    + "; set PYTHON or -Dftc.sim.python");
        }
        return python;
    }

    private static void assertTruncatedBagRejected() throws IOException {
        File truncated = File.createTempFile("robot-replay-truncated", ".jsonl");
        try {
            java.nio.file.Files.writeString(truncated.toPath(),
                    "{\"bag\":1,\"engine\":\"cplx1\","
                            + "\"start_pose\":{\"x\":72,\"y\":72,\"h\":0}}\n"
                            + "{\"seam\":\"logic\",\"t_ms\":0}\n");
            try {
                new ReplayController(truncated);
                throw new AssertionError("truncated replay bag was accepted");
            } catch (IllegalArgumentException expected) {
                // The incomplete logic seam must not silently become an idle replay.
            }
        } finally {
            if (!truncated.delete()) truncated.deleteOnExit();
        }
    }

    private static Mechanism m() {
        return Mechanism.DEFAULT;
    }

    private static List<PoseBits> readTruthFromBagRun(ReplayController replay,
                                                       Mechanism mechanism,
                                                       int port) throws Exception {
        List<PoseBits> truth = new ArrayList<>();
        try (SimHal hal = connect(mechanism, port, replay.initialPose());
             RobotLoop loop = RobotFactory.createWithController(
                     hal, mechanism, replay.engineName(), replay, 0, false)) {
            for (int i = 0; i < replay.tickCount(); i++) {
                loop.tick();
                truth.add(PoseBits.of(hal.truth()));
            }
        }
        return truth;
    }

    private static void assertTrueOpenBag(RobotLoop loop, File bag, Pose start) {
        if (!loop.openBag(bag, "ReplayIntegrationTest", start)) {
            throw new AssertionError("could not configure replay bag");
        }
    }

    private static SimHal connect(Mechanism mechanism, int port, Pose pose) throws IOException {
        return new SimHal(mechanism, "127.0.0.1", port, 20, 42L, pose, 5000);
    }

    private static final class TestLineController implements IController {
        private boolean sent;

        @Override
        public RequestBatch decide(Feedback feedback) {
            if (sent) return RequestBatch.idle();
            sent = true;
            return RequestBatch.of(Request.path(1, PathRequest.named("test-line")));
        }
    }

    private static final class PoseBits {
        private final long x;
        private final long y;
        private final long h;

        private PoseBits(long x, long y, long h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }

        static PoseBits of(Pose pose) {
            if (pose == null) throw new AssertionError("simulator omitted truth pose");
            return new PoseBits(Double.doubleToLongBits(pose.x()),
                    Double.doubleToLongBits(pose.y()),
                    Double.doubleToLongBits(pose.heading()));
        }

        @Override
        public boolean equals(Object other) {
            if (!(other instanceof PoseBits value)) return false;
            return x == value.x && y == value.y && h == value.h;
        }

        @Override
        public int hashCode() {
            return java.util.Objects.hash(x, y, h);
        }

        @Override
        public String toString() {
            return "PoseBits{" + x + "," + y + "," + h + "}";
        }
    }
}
