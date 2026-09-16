package boobuzz.core.controller.socket;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
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

    private static void waitForClient(SocketController controller) throws InterruptedException {
        for (int i = 0; i < 100 && !controller.clientConnected(); i++) {
            Thread.sleep(5);
        }
        assertTrue(controller.clientConnected());
    }
}
