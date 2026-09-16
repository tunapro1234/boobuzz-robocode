package boobuzz.core.debug;

import com.pedropathing.math.Pose;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Objects;

/** Writes a deterministic header followed by the three seam lines per tick. */
public final class BagWriter implements AutoCloseable {

    private final BufferedWriter writer;
    private final Path path;
    private int pendingLines;

    public BagWriter(Path path, String engine, String controller, String constantsHash)
            throws IOException {
        this(path, engine, controller, constantsHash, null);
    }

    public BagWriter(Path path, String engine, String controller, String constantsHash,
                     Pose startPose) throws IOException {
        this.path = Objects.requireNonNull(path, "bag path");
        Path parent = path.toAbsolutePath().getParent();
        if (parent != null) Files.createDirectories(parent);
        writer = Files.newBufferedWriter(path, StandardCharsets.UTF_8,
                StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING,
                StandardOpenOption.WRITE);
        LinkedHashMap<String, Object> header = new LinkedHashMap<>();
        header.put("bag", 1);
        header.put("engine", engine);
        header.put("controller", controller);
        header.put("constants_hash", constantsHash);
        header.put("tap_dropped", 0);
        if (startPose != null) {
            LinkedHashMap<String, Object> pose = new LinkedHashMap<>();
            pose.put("x", startPose.x());
            pose.put("y", startPose.y());
            pose.put("h", startPose.heading());
            header.put("start_pose", pose);
        }
        writer.write(JsonCodec.stringify(header));
        writer.newLine();
        writer.flush();
    }

    public Path path() {
        return path;
    }

    public synchronized void writeLines(List<String> lines) throws IOException {
        for (String line : lines) {
            writer.write(line);
            writer.newLine();
            pendingLines++;
        }
        if (pendingLines >= 96) {
            writer.flush();
            pendingLines = 0;
        }
    }

    public synchronized void flush() throws IOException {
        writer.flush();
        pendingLines = 0;
    }

    @Override
    public synchronized void close() {
        try {
            writer.flush();
            writer.close();
        } catch (IOException ignored) {
            // Shutdown should not mask the robot loop's result.
        }
    }
}
