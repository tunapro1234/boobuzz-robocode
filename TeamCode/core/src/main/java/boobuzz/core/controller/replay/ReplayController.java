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
        Pose headerPose = null;
        String loadedEngine = "cplx1";
        boolean headerSeen = false;
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(
                new FileInputStream(bagPath), StandardCharsets.UTF_8))) {
            String line;
            while ((line = reader.readLine()) != null) {
            if (line.trim().isEmpty()) continue;
            Map<String, Object> root = JsonCodec.parseObject(line);
            if (root.containsKey("bag")) {
                headerSeen = true;
                loadedEngine = JsonCodec.str(root, "engine", loadedEngine);
                Map<String, Object> start = JsonCodec.object(root, "start_pose");
                if (start.isEmpty()) {
                    throw new IOException("replay bag requires a start_pose reset pose");
                }
                headerPose = new Pose(JsonCodec.num(start, "x", 0.0),
                        JsonCodec.num(start, "y", 0.0),
                        JsonCodec.num(start, "h", 0.0));
                continue;
            }
            String seam = JsonCodec.str(root, "seam", "");
            if ("logic".equals(seam)) {
                loadedBatches.add(SeamJson.batchFrom(root));
            }
            }
        }
        if (!headerSeen || headerPose == null) {
            throw new IOException("replay bag is missing its start_pose header");
        }
        batches = Collections.unmodifiableList(new ArrayList<>(loadedBatches));
        // The header is the reset pose; the first HAL seam is already post-reset.
        initialPose = headerPose;
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
