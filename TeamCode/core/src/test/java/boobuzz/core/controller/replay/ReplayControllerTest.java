package boobuzz.core.controller.replay;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.debug.JsonCodec;
import boobuzz.core.debug.SeamJson;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public class ReplayControllerTest {

    @Test
    public void emitsBatchesInBagOrderAndUsesHeaderStartPose() throws Exception {
        Path bag = Files.createTempFile("replay-controller", ".jsonl");
        String header = JsonCodec.stringify(Map.of(
                "bag", 1, "engine", "direct",
                "start_pose", Map.of("x", 12.0, "y", 34.0, "h", 0.5)));
        String logic = SeamJson.logic(0, Feedback.of(
                new boobuzz.core.contract.WorldSnapshot(0, Pose.zero(), 0.0, 12.0)),
                RequestBatch.of(Request.shoot(7, 1)));
        Files.write(bag, List.of(header, logic));
        try {
            ReplayController replay = new ReplayController(bag.toFile());
            assertEquals("direct", replay.engineName());
            assertEquals(12.0, replay.initialPose().x(), 0.0);
            assertEquals(34.0, replay.initialPose().y(), 0.0);
            assertEquals(0.5, replay.initialPose().heading(), 0.0);
            assertEquals(1, replay.tickCount());
            assertEquals(7, replay.decide(null).requests().get(0).id());
            assertTrue(replay.isDone());
        } finally {
            Files.deleteIfExists(bag);
        }
    }

    @Test
    public void headerResetPoseWinsOverFirstHalPose() throws Exception {
        Path bag = Files.createTempFile("replay-header", ".jsonl");
        String header = JsonCodec.stringify(Map.of(
                "bag", 1, "engine", "cplx1",
                "start_pose", Map.of("x", 12.0, "y", 34.0, "h", 0.5)));
        RobotState firstHal = new RobotState(0, Map.of(), Map.of(), 0.0,
                new Pose(99.0, 88.0, 0.25), 12.0);
        String hal = SeamJson.hal(0, firstHal, RobotAction.zero());
        Files.write(bag, List.of(header, hal));
        try {
            ReplayController replay = new ReplayController(bag.toFile());
            assertEquals(12.0, replay.initialPose().x(), 0.0);
            assertEquals(34.0, replay.initialPose().y(), 0.0);
            assertEquals(0.5, replay.initialPose().heading(), 0.0);
        } finally {
            Files.deleteIfExists(bag);
        }
    }

    @Test
    public void legacyBagWithoutHeaderPoseIsRejected() throws Exception {
        Path bag = Files.createTempFile("replay-legacy", ".jsonl");
        String header = JsonCodec.stringify(Map.of("bag", 1, "engine", "cplx1"));
        Files.write(bag, List.of(header));
        try {
            assertThrows(IOException.class, () -> new ReplayController(bag.toFile()));
        } finally {
            Files.deleteIfExists(bag);
        }
    }
}
