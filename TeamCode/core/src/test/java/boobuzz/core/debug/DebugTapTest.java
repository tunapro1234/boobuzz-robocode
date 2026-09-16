package boobuzz.core.debug;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.WorldSnapshot;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class DebugTapTest {

    @Test
    public void broadcastsLinesWithoutBlockingThePublisher() throws Exception {
        int port;
        try (ServerSocket probe = new ServerSocket(0)) {
            port = probe.getLocalPort();
        }
        try (DebugTap tap = new DebugTap(port);
             Socket client = new Socket("127.0.0.1", port)) {
            client.setSoTimeout(2000);
            waitForClient(tap);
            tap.offer(frame(12));
            BufferedReader reader = new BufferedReader(new InputStreamReader(
                    client.getInputStream(), StandardCharsets.UTF_8));
            assertTrue(reader.readLine().contains("\"seam\":\"hal\""));
            assertTrue(tap.enabled());
            assertEquals(1, tap.clientCount());
        }
    }

    @Test
    public void zeroDisablesTheTap() throws Exception {
        try (DebugTap tap = new DebugTap(0)) {
            assertFalse(tap.enabled());
            tap.offer(frame(0));
            assertEquals(0, tap.clientCount());
        }
    }

    private static void waitForClient(DebugTap tap) throws InterruptedException {
        for (int i = 0; i < 100 && tap.clientCount() == 0; i++) {
            Thread.sleep(5);
        }
        assertEquals(1, tap.clientCount());
    }

    private static DebugFrame frame(long t) {
        RobotState state = new RobotState(t, Map.of(), Map.of(), 0.0,
                Pose.zero(), 12.0);
        Feedback feedback = Feedback.of(new WorldSnapshot(t, Pose.zero(), 0.0, 12.0));
        return new DebugFrame(state, RobotAction.zero(), java.util.List.of(), feedback,
                boobuzz.core.contract.RequestBatch.idle());
    }
}
