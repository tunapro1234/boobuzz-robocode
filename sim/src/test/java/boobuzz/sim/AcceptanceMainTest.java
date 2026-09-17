package boobuzz.sim;

import boobuzz.core.hal.Mechanism;

import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Focused Java-side proof of the A03 schedule without a lossy debug socket. */
public class AcceptanceMainTest {

    @Test
    public void cancelScenarioRunsThroughDirectSimHalAndEmitsTraceSchema() throws Exception {
        try (FakeSimServer server = new FakeSimServer(Mechanism.DEFAULT.motorNames())) {
            PrintStream original = System.out;
            ByteArrayOutputStream captured = new ByteArrayOutputStream();
            try {
                System.setOut(new PrintStream(captured, true, StandardCharsets.UTF_8));
                int status = AcceptanceMain.run(new String[] {
                        "--scenario", "A-cancel",
                        "--engine", "direct",
                        "--seed", "42",
                        "--port", Integer.toString(server.port()),
                        "--ticks", "101",
                        "--connect-timeout", "1000"
                });
                assertEquals(0, status);
            } finally {
                System.setOut(original);
            }
            String output = captured.toString(StandardCharsets.UTF_8);
            assertTrue(output.contains("\"scenario\":\"A-cancel\""));
            assertTrue(output.contains("\"request_statuses\":"));
            assertTrue(output.contains("ACCEPTANCE_RESULT {\"ok\":true"));
        }
    }
}
