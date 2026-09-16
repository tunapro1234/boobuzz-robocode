package boobuzz.core.controller.replay;

import boobuzz.core.controller.IController;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.debug.JsonCodec;
import boobuzz.core.debug.SeamJson;

import com.pedropathing.math.Pose;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/** Replays the logic seam batches from a JSONL bag, independent of live feedback. */
public final class ReplayController implements IController {

    private final List<RequestBatch> batches;
    private final Pose initialPose;
    private final String engineName;
    private int index;

    public ReplayController(File bagPath) throws IOException {
        List<RequestBatch> loadedBatches = new ArrayList<>();
        Pose loadedPose = null;
        Pose headerPose = null;
        String loadedEngine = "cplx1";
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(
                new FileInputStream(bagPath), StandardCharsets.UTF_8))) {
            String line;
            while ((line = reader.readLine()) != null) {
            if (line.trim().isEmpty()) continue;
            Map<String, Object> root = JsonCodec.parseObject(line);
            if (root.containsKey("bag")) {
                loadedEngine = JsonCodec.str(root, "engine", loadedEngine);
                Map<String, Object> start = JsonCodec.object(root, "start_pose");
                if (!start.isEmpty()) {
                    headerPose = new Pose(JsonCodec.num(start, "x", 0.0),
                            JsonCodec.num(start, "y", 0.0),
                            JsonCodec.num(start, "h", 0.0));
                }
                continue;
            }
            String seam = JsonCodec.str(root, "seam", "");
            if ("logic".equals(seam)) {
                loadedBatches.add(SeamJson.batchFrom(root));
            } else if ("hal".equals(seam) && loadedPose == null) {
                Map<String, Object> state = JsonCodec.object(root, "state");
                Map<String, Object> pinpoint = JsonCodec.object(state, "pinpoint");
                if (!pinpoint.isEmpty()) {
                    loadedPose = new Pose(JsonCodec.num(pinpoint, "x", 0.0),
                            JsonCodec.num(pinpoint, "y", 0.0),
                            JsonCodec.num(pinpoint, "h", 0.0));
                }
            }
            }
        }
        batches = Collections.unmodifiableList(new ArrayList<>(loadedBatches));
        initialPose = loadedPose == null ? headerPose : loadedPose;
        engineName = loadedEngine;
    }

    @Override
    public RequestBatch decide(Feedback feedback) {
        if (index >= batches.size()) {
            return RequestBatch.idle();
        }
        return batches.get(index++);
    }

    public int tickCount() {
        return batches.size();
    }

    public int index() {
        return index;
    }

    public boolean isDone() {
        return index >= batches.size();
    }

    public Pose initialPose() {
        return initialPose;
    }

    public String engineName() {
        return engineName;
    }
}
