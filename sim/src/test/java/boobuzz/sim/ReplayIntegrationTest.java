package boobuzz.sim;

import boobuzz.core.RobotFactory;
import boobuzz.core.RobotLoop;
import boobuzz.core.controller.replay.ReplayController;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.hal.Mechanism;

import com.pedropathing.math.Pose;

import org.junit.Assume;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/** Real Python-server record/replay proof for the deterministic seam contract. */
public class ReplayIntegrationTest {

    @Test
    public void seededRecordAndReplayAreBitEqual() throws Exception {
        File simulator = new File("/home/shared/projects/boobuzz/re-cock-nize");
        File python = new File(simulator, ".venv/bin/python");
        File root = new File(System.getProperty("user.dir"));
        if ("sim".equals(root.getName())) root = root.getParentFile();
        File mechanism = new File(root,
                "TeamCode/core/src/main/java/boobuzz/core/hal/RobotConstants.java");
        Assume.assumeTrue("re-cock-nize venv is required for this integration test",
                python.isFile() && mechanism.isFile());

        File bag = File.createTempFile("robot-replay", ".jsonl");
        int recordPort = freePort(5580);
        int replayPort = freePort(recordPort + 1);
        Process recordServer = startServer(simulator, python, mechanism, recordPort);
        List<PoseBits> recordedTruth = new ArrayList<>();
        try {
            Pose start = new Pose(12.0, 34.0, 0.25);
            Mechanism m = Mechanism.DEFAULT;
            try (SimHal hal = connect(m, recordPort, start);
                 RobotLoop loop = RobotFactory.create(hal, m, "cplx1",
                         RequestStream.manual(0.25, 0.0, 0.0), 0, true)) {
                assertTrueOpenBag(loop, bag, start);
                for (int i = 0; i < 80; i++) {
                    loop.tick();
                    recordedTruth.add(PoseBits.of(hal.truth()));
                }
            }
        } finally {
            stopServer(recordServer);
        }

        Process replayServer = startServer(simulator, python, mechanism, replayPort);
        try {
            ReplayController replay = new ReplayController(bag);
            assertEquals(80, replay.tickCount());
            assertNotNull(replay.initialPose());
            List<PoseBits> replayedTruth = readTruthFromBagRun(replay, m(), replayPort);
            assertEquals("truth sequence length", recordedTruth.size(), replayedTruth.size());
            for (int i = 0; i < recordedTruth.size(); i++) {
                assertEquals("truth tick " + i, recordedTruth.get(i), replayedTruth.get(i));
            }
        } finally {
            stopServer(replayServer);
            if (!bag.delete()) bag.deleteOnExit();
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

    private static int freePort(int minimum) throws IOException {
        for (int port = minimum; port < 65535; port++) {
            try (ServerSocket probe = new ServerSocket()) {
                probe.setReuseAddress(true);
                probe.bind(new InetSocketAddress("127.0.0.1", port));
                return port;
            } catch (IOException ignored) {
                // Try the next port in the requested range.
            }
        }
        throw new IOException("no free simulator port");
    }

    private static Process startServer(File simulator, File python, File mechanism, int port)
            throws Exception {
        Process process = new ProcessBuilder(
                python.getAbsolutePath(), "-m", "sim.server",
                "--mechanism", mechanism.getAbsolutePath(),
                "--physics", "pymunk", "--headless", "--port", Integer.toString(port))
                .directory(simulator)
                .redirectErrorStream(true)
                .start();
        Thread output = new Thread(() -> {
            try {
                while (process.getInputStream().read() >= 0) {
                    // Drain server diagnostics so a long integration cannot block it.
                }
            } catch (IOException ignored) {
                // Process teardown closes the stream.
            }
        }, "replay-test-server-output-" + port);
        output.setDaemon(true);
        output.start();
        long deadline = System.nanoTime() + 10_000_000_000L;
        while (System.nanoTime() < deadline) {
            if (!process.isAlive()) {
                throw new IOException("simulator exited before listening on " + port);
            }
            try (Socket socket = new Socket()) {
                socket.connect(new InetSocketAddress("127.0.0.1", port), 100);
                return process;
            } catch (IOException ignored) {
                Thread.sleep(25);
            }
        }
        stopServer(process);
        throw new IOException("simulator did not listen on " + port);
    }

    private static void stopServer(Process process) throws InterruptedException {
        if (process == null) return;
        process.destroy();
        if (!process.waitFor(2, java.util.concurrent.TimeUnit.SECONDS)) {
            process.destroyForcibly();
            process.waitFor(2, java.util.concurrent.TimeUnit.SECONDS);
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
