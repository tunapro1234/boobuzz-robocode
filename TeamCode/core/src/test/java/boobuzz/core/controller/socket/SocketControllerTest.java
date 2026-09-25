package boobuzz.core.controller.socket;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.logic.IRobotEngine;
import boobuzz.core.logic.cplx1.CplxEngine1;
import boobuzz.core.logic.direct.DirectEngine;
import boobuzz.core.subsystem.IDrive;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.stub.StubIntake;
import boobuzz.core.subsystem.stub.StubShooter;
import boobuzz.core.debug.JsonCodec;
import boobuzz.core.debug.SeamJson;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.Map;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class SocketControllerTest {

    @Test
    public void receivesBatchAndEchoesFeedbackOffTheLoopThread() throws Exception {
        int port;
        try (ServerSocket probe = new ServerSocket(0)) {
            port = probe.getLocalPort();
        }
        try (SocketController controller = new SocketController(port, 250);
             Socket client = new Socket("127.0.0.1", port)) {
            client.setSoTimeout(2000);
            waitForClient(controller);
            BufferedWriter out = new BufferedWriter(new OutputStreamWriter(
                    client.getOutputStream(), StandardCharsets.UTF_8));
            BufferedReader in = new BufferedReader(new InputStreamReader(
                    client.getInputStream(), StandardCharsets.UTF_8));

            RequestBatch batch = RequestBatch.of(Request.path(7, PathRequest.named("test-line")));
            out.write(JsonCodec.stringify(SeamJson.batchMap(batch)));
            out.newLine();
            out.flush();

            RequestBatch received = RequestBatch.idle();
            for (int i = 0; i < 100; i++) {
                received = controller.decide(null);
                if (!received.requests().isEmpty()) break;
                Thread.sleep(5);
            }
            assertEquals(7, received.requests().get(0).id());
            assertEquals("test-line", received.requests().get(0).path().pathId());

            Feedback feedback = Feedback.of(new boobuzz.core.contract.WorldSnapshot(
                    12, new Pose(3, 4, 0.5), 0.5, 12.0));
            controller.decide(feedback);
            Map<String, Object> echo = JsonCodec.parseObject(in.readLine());
            assertEquals("feedback", JsonCodec.str(echo, "type", ""));
            assertEquals(12.0, JsonCodec.num(echo, "t_ms", 0.0), 0.0);

            Thread.sleep(300);
            assertTrue(controller.decide(null).requests().isEmpty());
        }
    }

    @Test
    public void timeoutCancelsAnActivePathInBothEngines() throws Exception {
        assertTimeoutStops(new DirectEngine(new Subsystems(
                new RecordingDrive(), new StubShooter(), new StubIntake())));
        assertTimeoutStops(new CplxEngine1(new Subsystems(
                new RecordingDrive(), new StubShooter(), new StubIntake())));
    }

    @Test
    public void malformedBatchesDoNotRefreshTheWatchdog() throws Exception {
        int port;
        try (ServerSocket probe = new ServerSocket(0)) {
            port = probe.getLocalPort();
        }
        try (SocketController controller = new SocketController(port, 60);
             Socket client = new Socket("127.0.0.1", port)) {
            waitForClient(controller);
            BufferedWriter out = new BufferedWriter(new OutputStreamWriter(
                    client.getOutputStream(), StandardCharsets.UTF_8));
            out.write(JsonCodec.stringify(SeamJson.batchMap(
                    RequestBatch.of(Request.path(12, PathRequest.named("test-line"))))));
            out.newLine();
            out.flush();
            RequestBatch first = RequestBatch.idle();
            for (int i = 0; i < 100 && first.requests().isEmpty(); i++) {
                first = controller.decide(null);
                if (first.requests().isEmpty()) Thread.sleep(2);
            }
            assertEquals(12, first.requests().get(0).id());

            String[] malformed = {
                    "{",
                    "{}",
                    "{\"stream\":[] ,\"requests\":[],\"cancels\":[]}",
                    "{\"stream\":{\"vx\":0,\"vy\":0,\"omega\":0,\"manualDrive\":true},"
                            + "\"requests\":[{\"id\":1,\"type\":\"NOPE\",\"params\":[],\"path\":null}],"
                            + "\"cancels\":[]}",
                    "{\"stream\":{\"vx\":\"fast\",\"vy\":0,\"omega\":0,\"manualDrive\":true},"
                            + "\"requests\":[],\"cancels\":[]}"};
            for (String line : malformed) {
                out.write(line);
                out.newLine();
                out.flush();
                Thread.sleep(5);
            }
            Thread.sleep(90);
            RequestBatch stopped = controller.decide(null);
            assertTrue("watchdog must cancel the stale path",
                    stopped.cancels().length == 1
                            && stopped.cancels()[0] == RequestBatch.CANCEL_ALL);
            assertTrue(controller.inputError() != null);
        }
    }

    @Test
    public void requestsAndCancelsAreDeliveredOnceWhileTheStreamPersists() throws Exception {
        int port;
        try (ServerSocket probe = new ServerSocket(0)) {
            port = probe.getLocalPort();
        }
        try (SocketController controller = new SocketController(port, 1000);
             Socket client = new Socket("127.0.0.1", port)) {
            waitForClient(controller);
            BufferedWriter out = new BufferedWriter(new OutputStreamWriter(
                    client.getOutputStream(), StandardCharsets.UTF_8));
            // Two lines before any tick: both edges survive, in arrival order,
            // and the newer stream wins.
            out.write(JsonCodec.stringify(SeamJson.batchMap(new RequestBatch(
                    RequestStream.manual(0.25, 0.0, 0.0),
                    List.of(Request.path(7, PathRequest.named("test-line"))),
                    new int[] {3}))));
            out.newLine();
            out.write(JsonCodec.stringify(SeamJson.batchMap(new RequestBatch(
                    RequestStream.manual(0.5, 0.0, 0.0),
                    List.of(Request.shoot(8, 1)), new int[] {4}))));
            out.newLine();
            out.flush();
            waitForBatches(controller, 2);

            RequestBatch first = controller.decide(null);
            assertEquals(2, first.requests().size());
            assertEquals(7, first.requests().get(0).id());
            assertEquals(8, first.requests().get(1).id());
            assertEquals(2, first.cancels().length);
            assertEquals(3, first.cancels()[0]);
            assertEquals(4, first.cancels()[1]);
            assertEquals(0.5, first.stream().vx(), 0.0);

            for (int tick = 0; tick < 3; tick++) {
                RequestBatch later = controller.decide(null);
                assertTrue("a request is an edge", later.requests().isEmpty());
                assertEquals("a cancel is an edge", 0, later.cancels().length);
                assertTrue(later.stream().manualDrive());
                assertEquals("the stream is a level", 0.5, later.stream().vx(), 0.0);
            }
        }
    }

    @Test
    public void freePortFormReportsTheBoundPortWhilePortZeroStaysDisabled() throws Exception {
        try (SocketController disabled = new SocketController(0, 250)) {
            assertEquals(0, disabled.port());
            assertTrue(!disabled.enabled());
        }
        try (SocketController controller = SocketController.onFreePort(250);
             Socket client = new Socket("127.0.0.1", controller.port())) {
            assertTrue(controller.port() > 0);
            assertTrue(controller.enabled());
            waitForClient(controller);
        }
    }

    @Test
    public void closeJoinsSocketThreadsWithinBound() throws Exception {
        int port;
        try (ServerSocket probe = new ServerSocket(0)) {
            port = probe.getLocalPort();
        }
        SocketController controller = new SocketController(port, 60);
        Socket client = new Socket("127.0.0.1", port);
        try {
            waitForClient(controller);
        } finally {
            client.close();
            controller.close();
        }
        assertThreadsGone("control-socket-accept-", 1000);
        assertThreadsGone("control-socket-reader-", 1000);
        assertThreadsGone("control-socket-writer-", 1000);
        assertThreadsGone("control-socket-writer-watchdog-", 1000);
        assertThreadsGone("control-socket-feedback", 1000);
    }

    @Test
    public void slowFeedbackClientCannotStarveAReconnect() throws Exception {
        int port;
        try (ServerSocket probe = new ServerSocket(0)) {
            port = probe.getLocalPort();
        }
        try (SocketController controller = new SocketController(port, 1000);
             Socket slow = new Socket("127.0.0.1", port)) {
            slow.setReceiveBufferSize(128);
            waitForClient(controller);
            List<RequestStatus> statuses = new java.util.ArrayList<>();
            for (int i = 0; i < 80; i++) {
                statuses.add(new RequestStatus(i, RequestStatus.State.ACTIVE,
                        0.0, "feedback-status-padding"));
            }
            Feedback feedback = new Feedback(
                    new boobuzz.core.contract.WorldSnapshot(9, Pose.zero(), 0.0, 12.0),
                    statuses, 9);
            long start = System.nanoTime();
            for (int i = 0; i < 5000; i++) {
                controller.decide(feedback);
            }
            long elapsedMs = (System.nanoTime() - start) / 1_000_000L;
            assertTrue("feedback publication must remain bounded", elapsedMs < 500);
            for (int i = 0; i < 100 && controller.feedbackDrops() == 0; i++) {
                Thread.sleep(5);
            }
            assertTrue("slow feedback pressure should be accounted for",
                    controller.feedbackDrops() > 0);

            slow.close();
            for (int i = 0; i < 100 && controller.clientConnected(); i++) {
                Thread.sleep(5);
            }
            try (Socket fast = new Socket("127.0.0.1", port)) {
                fast.setSoTimeout(2000);
                waitForClient(controller);
                controller.decide(feedback);
                BufferedReader reader = new BufferedReader(new InputStreamReader(
                        fast.getInputStream(), StandardCharsets.UTF_8));
                Map<String, Object> echo = JsonCodec.parseObject(reader.readLine());
                assertEquals("feedback", JsonCodec.str(echo, "type", ""));
            }
        }
    }

    private static void assertTimeoutStops(IRobotEngine engine) throws Exception {
        RecordingDrive drive = (RecordingDrive) (engine instanceof DirectEngine
                ? ((DirectEngine) engine).subsystems().drive()
                : ((CplxEngine1) engine).subsystems().drive());
        int port;
        try (ServerSocket probe = new ServerSocket(0)) {
            port = probe.getLocalPort();
        }
        try (SocketController controller = new SocketController(port, 40);
             Socket client = new Socket("127.0.0.1", port)) {
            waitForClient(controller);
            BufferedWriter out = new BufferedWriter(new OutputStreamWriter(
                    client.getOutputStream(), StandardCharsets.UTF_8));
            RequestBatch path = RequestBatch.of(Request.path(91,
                    PathRequest.named("test-line")));
            out.write(JsonCodec.stringify(SeamJson.batchMap(path)));
            out.newLine();
            out.flush();
            // Wait without calling decide: the request is delivered to one tick only.
            waitForBatches(controller, 1);

            TestHal hal = new TestHal();
            boobuzz.core.RobotLoop loop = new boobuzz.core.RobotLoop(hal, engine, controller);
            loop.tick();
            assertTrue("path should drive before timeout", drive.following);
            Thread.sleep(80);
            loop.tick();
            assertTrue("watchdog must stop the active path", drive.stopped);
            assertEquals(0.0, hal.last.motor("fl"), 1e-9);
            assertTrue("watchdog must emit one cancel-all batch",
                    controller.decide(null).cancels().length == 0);
            loop.close();
        }
    }

    private static void waitForBatches(SocketController controller, long count)
            throws InterruptedException {
        for (int i = 0; i < 200 && controller.batchesReceived() < count; i++) {
            Thread.sleep(5);
        }
        assertEquals(count, controller.batchesReceived());
    }

    private static void waitForClient(SocketController controller) throws InterruptedException {
        for (int i = 0; i < 100 && !controller.clientConnected(); i++) {
            Thread.sleep(5);
        }
        assertTrue(controller.clientConnected());
    }

    private static final class TestHal implements boobuzz.core.hal.IHal {
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

        @Override public boobuzz.core.contract.GamepadState get() {
            return boobuzz.core.contract.GamepadState.neutral();
        }
    }

    private static final class RecordingDrive implements IDrive {
        private boolean following;
        private boolean stopped = true;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {
            if (following && !stopped) out.motor("fl", 1.0);
        }
        @Override public void manual(double vx, double vy, double omega) {
            stopped = false;
            following = false;
        }
        @Override public void follow(PathRequest request) {
            following = true;
            stopped = false;
        }
        @Override public void stop() {
            stopped = true;
            following = false;
        }
        @Override public boolean pathDone() { return false; }
        @Override public Pose pose() { return Pose.zero(); }
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
