package boobuzz.sim;

import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;
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

        try (ChildProcess server = ChildProcess.startSimServer(simulator, python, mechanism)) {
            int simPort = server.awaitPort(0L);
            String java = new File(System.getProperty("java.home"), "bin/java")
                    .getAbsolutePath();
            // --control-port 0: SimMain binds a free port and prints it.
            try (ChildProcess sim = ChildProcess.start(List.of(
                    java, "-cp", System.getProperty("java.class.path"),
                    "boobuzz.sim.SimMain",
                    "--host", "127.0.0.1", "--port", Integer.toString(simPort),
                    "--controller", "socket", "--control-port", "0",
                    "--control-timeout", "1000", "--engine", "cplx1",
                    "--steps", "10000", "--dt", "20", "--seed", "42",
                    "--tap-port", "0"),
                    root, ChildProcess.SIM_MAIN_CONTROL_PORT, "sim-main")) {
                int controlPort = sim.awaitPort(10_000L);

                try (ChildProcess agentProcess = ChildProcess.start(List.of(
                        python.getAbsolutePath(), agent.getAbsolutePath(),
                        "--host", "127.0.0.1", "--port", Integer.toString(controlPort),
                        "--timeout", "30"),
                        root, null, "agent")) {
                    String agentOutput = agentProcess.awaitExitOutput(30_000L);
                    assertEquals("agent output: " + agentOutput, 0,
                            agentProcess.process().exitValue());
                }

                String simOutput = sim.awaitExitOutput(30_000L);
                assertEquals("SimMain output: " + simOutput, 0, sim.process().exitValue());
                Matcher pose = FINAL_POSE.matcher(simOutput);
                assertTrue("missing final pose in SimMain output:\n" + simOutput, pose.find());
                assertEquals(120.0, Double.parseDouble(pose.group(1)), 1.5);
                assertEquals(72.0, Double.parseDouble(pose.group(2)), 1.5);
            }
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
}
