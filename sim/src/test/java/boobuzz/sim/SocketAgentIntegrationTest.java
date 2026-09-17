package boobuzz.sim;

import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Runs the stdlib external controller against SimMain and real pymunk physics. */
public class SocketAgentIntegrationTest {

    private static final Pattern FINAL_POSE = Pattern.compile(
            "final pose: x=([-+0-9.]+) y=([-+0-9.]+) h=([-+0-9.]+) rad");

    @Test
    public void agentDrivesGotoShootsAndStopsAtTestLine() throws Exception {
        File root = repositoryRoot();
        File simulator = simulatorRoot(root);
        File python = pythonExecutable(simulator);
        File mechanism = new File(root,
                "TeamCode/core/src/main/java/boobuzz/core/hal/RobotConstants.java");
        File agent = new File(root, "tools/agent_example.py");
        assertTrue("RobotConstants.java is missing", mechanism.isFile());
        assertTrue("tools/agent_example.py is missing", agent.isFile());

        int simPort = freePort(5580);
        int controlPort = freePort(5601);
        Process server = startServer(simulator, python, mechanism, simPort);
        Process sim = null;
        Process agentProcess = null;
        try {
            String java = new File(System.getProperty("java.home"), "bin/java")
                    .getAbsolutePath();
            sim = new ProcessBuilder(
                    java, "-cp", System.getProperty("java.class.path"),
                    "boobuzz.sim.SimMain",
                    "--host", "127.0.0.1", "--port", Integer.toString(simPort),
                    "--controller", "socket", "--control-port", Integer.toString(controlPort),
                    "--control-timeout", "1000", "--engine", "cplx1",
                    "--steps", "10000", "--dt", "20", "--seed", "42",
                    "--tap-port", "0")
                    .directory(root)
                    .redirectErrorStream(true)
                    .start();
            waitForPort(controlPort, sim, 10_000L);

            agentProcess = new ProcessBuilder(
                    python.getAbsolutePath(), agent.getAbsolutePath(),
                    "--host", "127.0.0.1", "--port", Integer.toString(controlPort),
                    "--timeout", "30")
                    .directory(root)
                    .redirectErrorStream(true)
                    .start();
            assertTrue("agent timed out", agentProcess.waitFor(30, TimeUnit.SECONDS));
            String agentOutput = new String(agentProcess.getInputStream().readAllBytes(),
                    StandardCharsets.UTF_8);
            assertEquals("agent output: " + agentOutput, 0, agentProcess.exitValue());

            assertTrue("SimMain timed out", sim.waitFor(30, TimeUnit.SECONDS));
            String simOutput = new String(sim.getInputStream().readAllBytes(),
                    StandardCharsets.UTF_8);
            assertEquals("SimMain output: " + simOutput, 0, sim.exitValue());
            Matcher pose = FINAL_POSE.matcher(simOutput);
            assertTrue("missing final pose in SimMain output:\n" + simOutput, pose.find());
            assertEquals(120.0, Double.parseDouble(pose.group(1)), 1.5);
            assertEquals(72.0, Double.parseDouble(pose.group(2)), 1.5);
        } finally {
            stopProcess(agentProcess);
            stopProcess(sim);
            stopProcess(server);
        }
    }

    private static File repositoryRoot() throws IOException {
        File root = new File(System.getProperty("user.dir")).getAbsoluteFile();
        if ("sim".equals(root.getName())) root = root.getParentFile();
        return root.getCanonicalFile();
    }

    private static File simulatorRoot(File root) throws IOException {
        String configured = System.getProperty("ftc.sim.root");
        if (configured == null || configured.trim().isEmpty()) {
            configured = System.getenv("FTC_SIM_ROOT");
        }
        File simulator = configured == null || configured.trim().isEmpty()
                ? new File(root.getParentFile(), "re-cock-nize")
                : new File(configured.trim());
        simulator = simulator.getCanonicalFile();
        if (!simulator.isDirectory()) {
            throw new AssertionError("simulator root is missing: " + simulator
                    + "; set FTC_SIM_ROOT or -Dftc.sim.root");
        }
        return simulator;
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
        throw new IOException("no free port");
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
        waitForPort(port, process, 10_000L);
        return process;
    }

    private static void waitForPort(int port, Process process, long timeoutMs)
            throws Exception {
        long deadline = System.nanoTime() + timeoutMs * 1_000_000L;
        while (System.nanoTime() < deadline) {
            if (!process.isAlive()) {
                throw new IOException("process exited before listening on " + port);
            }
            try (Socket socket = new Socket()) {
                socket.connect(new InetSocketAddress("127.0.0.1", port), 100);
                return;
            } catch (IOException ignored) {
                Thread.sleep(25);
            }
        }
        throw new IOException("process did not listen on " + port);
    }

    private static void stopProcess(Process process) throws InterruptedException {
        if (process == null) return;
        if (!process.isAlive()) return;
        process.destroy();
        if (!process.waitFor(2, TimeUnit.SECONDS)) {
            process.destroyForcibly();
            process.waitFor(2, TimeUnit.SECONDS);
        }
    }
}
