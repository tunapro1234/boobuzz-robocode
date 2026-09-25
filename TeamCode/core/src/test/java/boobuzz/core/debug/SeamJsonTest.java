package boobuzz.core.debug;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RequestType;
import boobuzz.core.contract.WorldSnapshot;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class SeamJsonTest {

    @Test
    public void seamLinesHaveTheDocumentedShape() {
        var state = new boobuzz.core.contract.RobotState(12, Map.of("fl", 4),
                Map.of("fl", 1.5), 0.2, new Pose(3, 4, 0.5), 12.4);
        var action = new RobotAction.Builder().motor("fl", 0.25).build();
        var feedback = Feedback.of(new WorldSnapshot(12, new Pose(3, 4, 0.5), 0.2, 12.4));
        var batch = new RequestBatch(RequestStream.manual(1, 2, 3),
                List.of(Request.shoot(7, 3)), new int[] {9});

        Map<String, Object> hal = JsonCodec.parseObject(SeamJson.hal(12, state, action));
        Map<String, Object> logic = JsonCodec.parseObject(SeamJson.logic(12, feedback, batch));
        assertEquals("hal", JsonCodec.str(hal, "seam", ""));
        assertTrue(JsonCodec.object(hal, "state").containsKey("pinpoint"));
        assertEquals("logic", JsonCodec.str(logic, "seam", ""));
        assertTrue(JsonCodec.object(logic, "batch").containsKey("stream"));
    }

    @Test
    public void requestBatchRoundTripsPathPayloads() {
        PathRequest path = new PathRequest(List.of(PathRequest.line(new Pose(10, 20))),
                PathRequest.Heading.constant(0.5), true, 24.0,
                new PathRequest.Braking(0.2, 0.8));
        Request original = Request.path(4, path);
        RequestBatch decoded = SeamJson.batchFrom(JsonCodec.parseObject(
                SeamJson.logic(5, Feedback.of(new WorldSnapshot(5, Pose.zero(), 0, 12)),
                        RequestBatch.of(original))));
        Request result = decoded.requests().get(0);
        assertEquals(original.id(), result.id());
        assertEquals(RequestType.PATH, result.type());
        assertEquals(1, result.path().segments().size());
        assertEquals(10.0, result.path().segments().get(0).end().x(), 1e-9);
        assertFalse(decoded.stream().manualDrive());
    }

    @Test
    public void phase11aAddendumRequestsRoundTrip() {
        List<Request> originals = List.of(
                Request.setShotPreset(1, 4000.0, 45.0, -0.25),
                Request.stopShooting(2),
                Request.mechanismRecovery(3, 2));
        RequestBatch decoded = SeamJson.batchFrom(JsonCodec.parseObject(JsonCodec.stringify(
                SeamJson.batchMap(new RequestBatch(RequestStream.idle(), originals, new int[0])))));
        assertEquals(originals.size(), decoded.requests().size());
        for (int i = 0; i < originals.size(); i++) {
            Request original = originals.get(i);
            Request result = decoded.requests().get(i);
            assertEquals(original.id(), result.id());
            assertEquals(original.type(), result.type());
            assertEquals(original.params().length, result.params().length);
            for (int p = 0; p < original.params().length; p++) {
                assertEquals(original.params()[p], result.params()[p], 0.0);
            }
        }
        assertEquals(RequestType.SET_SHOT_PRESET, decoded.requests().get(0).type());
        assertEquals(RequestType.STOP_SHOOTING, decoded.requests().get(1).type());
        assertEquals(RequestType.MECHANISM_RECOVERY, decoded.requests().get(2).type());
    }

    @Test
    public void everyRequestTypeNameDecodes() {
        for (RequestType type : RequestType.values()) {
            if (type == RequestType.PATH) {
                continue;
            }
            RequestBatch decoded = SeamJson.batchFrom(JsonCodec.parseObject(JsonCodec.stringify(
                    SeamJson.batchMap(RequestBatch.of(Request.of(5, type, 1.0))))));
            assertEquals(type, decoded.requests().get(0).type());
        }
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsMissingBatchFields() {
        SeamJson.batchFrom(Map.of());
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsNonFiniteStreamValues() {
        SeamJson.batchFrom(Map.of(
                "stream", Map.of("vx", Double.NaN, "vy", 0.0,
                        "omega", 0.0, "manualDrive", true),
                "requests", List.of(), "cancels", List.of()));
    }
}
