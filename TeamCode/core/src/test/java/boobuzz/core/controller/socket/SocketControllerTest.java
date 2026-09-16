package boobuzz.core.controller.socket;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
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
            for (int i = 0; i < 100 && controller.decide(null).requests().isEmpty(); i++) {
                Thread.sleep(2);
            }

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
}
