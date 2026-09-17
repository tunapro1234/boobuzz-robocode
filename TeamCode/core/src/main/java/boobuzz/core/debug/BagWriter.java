package boobuzz.core.debug;

import com.pedropathing.math.Pose;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/** Writes a deterministic header followed by the three seam lines per tick. */
public final class BagWriter implements AutoCloseable {

    private final BufferedWriter writer;
    private final File path;
    private int pendingLines;
    private long lastTimestampMs;
    private boolean closed;

    public BagWriter(File path, String engine, String controller, String constantsHash)
            throws IOException {
        this(path, engine, controller, constantsHash, null);
    }

    public BagWriter(File path, String engine, String controller, String constantsHash,
                     Pose startPose) throws IOException {
        this.path = Objects.requireNonNull(path, "bag path");
        File parent = path.getAbsoluteFile().getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs() && !parent.isDirectory()) {
            throw new IOException("could not create bag directory: " + parent);
        }
        writer = new BufferedWriter(new OutputStreamWriter(
                new FileOutputStream(path, false), StandardCharsets.UTF_8));
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

    public File path() {
        return path;
    }

    public synchronized void writeLines(List<String> lines) throws IOException {
        for (String line : lines) {
            writer.write(line);
            writer.newLine();
            pendingLines++;
            Map<String, Object> record = JsonCodec.parseObject(line);
            Object timestamp = record.get("t_ms");
            if (timestamp instanceof Number number) {
                lastTimestampMs = number.longValue();
            }
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

    /** Writes a seam-shaped footer and returns a terminal I/O error, if one occurred. */
    public synchronized String closeWithTapDrops(long tapDrops) {
        if (closed) {
            return null;
        }
        String error = null;
        try {
            if (tapDrops > 0) {
                LinkedHashMap<String, Object> footer = new LinkedHashMap<>();
                footer.put("seam", "meta");
                footer.put("t_ms", lastTimestampMs);
                footer.put("tap_dropped", tapDrops);
                writer.write(JsonCodec.stringify(footer));
                writer.newLine();
            }
            writer.flush();
            writer.close();
        } catch (IOException e) {
            error = e.getMessage() == null ? e.toString() : e.getMessage();
            try {
                writer.close();
            } catch (IOException ignored) {
                // Preserve the first terminal failure.
            }
        } finally {
            closed = true;
        }
        return error;
    }

    @Override
    public synchronized void close() {
        closeWithTapDrops(0);
    }
}
