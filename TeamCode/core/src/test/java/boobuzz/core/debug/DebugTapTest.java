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
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;

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

    @Test
    public void slowClientDoesNotBlockFastClientOrPublisher() throws Exception {
        int port;
        try (ServerSocket probe = new ServerSocket(0)) {
            port = probe.getLocalPort();
        }
        try (DebugTap tap = new DebugTap(port);
             Socket slow = new Socket("127.0.0.1", port);
             Socket fast = new Socket("127.0.0.1", port)) {
            slow.setReceiveBufferSize(1024);
            fast.setSoTimeout(2000);
            waitForClients(tap, 2);
            BufferedReader reader = new BufferedReader(new InputStreamReader(
                    fast.getInputStream(), StandardCharsets.UTF_8));
            AtomicInteger fastLines = new AtomicInteger();
            Thread drain = new Thread(() -> {
                try {
                    while (reader.readLine() != null) {
                        fastLines.incrementAndGet();
                    }
                } catch (Exception ignored) {
                    // The test closes the socket after the bounded assertion.
                }
            });
            drain.start();

            long start = System.nanoTime();
            for (int i = 0; i < 8_000; i++) {
                tap.offer(frame(i));
            }
            long elapsedMs = (System.nanoTime() - start) / 1_000_000L;

            assertTrue("publisher must remain non-blocking", elapsedMs < 500);
            for (int i = 0; i < 100 && fastLines.get() < 3; i++) {
                Thread.sleep(5);
            }
            assertTrue("fast client did not receive a complete frame", fastLines.get() >= 3);
            assertTrue("slow-client pressure should be accounted for", tap.droppedCount() > 0);
            fast.close();
            slow.close();
            drain.join(1000);
        }
    }

    @Test
    public void closeJoinsTapThreadsWithinBound() throws Exception {
        int port;
        try (ServerSocket probe = new ServerSocket(0)) {
            port = probe.getLocalPort();
        }
        DebugTap tap = new DebugTap(port);
        Socket client = new Socket("127.0.0.1", port);
        try {
            waitForClient(tap);
        } finally {
            client.close();
            tap.close();
        }
        assertThreadsGone("debug-tap-accept-", 1000);
        assertThreadsGone("debug-tap-dispatch", 1000);
        assertThreadsGone("debug-tap-client-", 1000);
        assertThreadsGone("debug-tap-client-watchdog-", 1000);
    }

    @Test
    public void reportsBagOpenFailureAfterConfigurationOnClose() throws Exception {
        Path path = Files.createTempFile("robot-bag-late-failure", ".jsonl");
        try (DebugTap tap = new DebugTap(0)) {
            tap.configureBag(path.toFile(), "cplx1", "test", "abc", Pose.zero());
            Files.delete(path);
            Files.createDirectory(path);
            tap.offer(frame(20));
            for (int i = 0; i < 100 && tap.bagError() == null; i++) {
                Thread.sleep(5);
            }
            tap.close();
            assertTrue("late bag open failure must be retained", tap.bagError() != null);
        } finally {
            Files.deleteIfExists(path);
        }
    }

    private static void waitForClient(DebugTap tap) throws InterruptedException {
        waitForClients(tap, 1);
    }

    private static void waitForClients(DebugTap tap, int expected) throws InterruptedException {
        for (int i = 0; i < 100 && tap.clientCount() < expected; i++) {
            Thread.sleep(5);
        }
        assertEquals(expected, tap.clientCount());
    }

    private static DebugFrame frame(long t) {
        RobotState state = new RobotState(t, Map.of(), Map.of(), 0.0,
                Pose.zero(), 12.0);
        Feedback feedback = Feedback.of(new WorldSnapshot(t, Pose.zero(), 0.0, 12.0));
        return new DebugFrame(state, RobotAction.zero(), java.util.List.of(), feedback,
                boobuzz.core.contract.RequestBatch.idle());
    }

    private static void assertThreadsGone(String prefix, long timeoutMs)
            throws InterruptedException {
        long deadline = System.nanoTime() + timeoutMs * 1_000_000L;
        while (System.nanoTime() < deadline) {
            boolean alive = false;
            for (Thread thread : Thread.getAllStackTraces().keySet()) {
                if (thread.isAlive() && thread.getName().startsWith(prefix)) {
                    alive = true;
                    break;
                }
            }
            if (!alive) return;
            Thread.sleep(5);
        }
        assertTrue("thread still alive: " + prefix, noLiveThread(prefix));
    }

    private static boolean noLiveThread(String prefix) {
        for (Thread thread : Thread.getAllStackTraces().keySet()) {
            if (thread.isAlive() && thread.getName().startsWith(prefix)) return false;
        }
        return true;
    }
}
