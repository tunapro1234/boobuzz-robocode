package boobuzz.sim;


import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.Closeable;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Minimal replacement for the Python server. FOR TESTS ONLY.
 *
 * <p>There is no real physics: encoders accumulate from power, and the pose is
 * advanced with simple mecanum forward kinematics and Euler integration. Its
 * purpose is protocol validation, not physics. Real integration uses re-cock-nize.
 */
final class FakeSimServer implements Closeable {

    private final ServerSocket server;
    private final Thread thread;
    private final List<String> motorNames;
    private volatile boolean running = true;
    private volatile boolean omitReadyState = false;
    private volatile String gamepadJson =
            "{\"lx\":0,\"ly\":0,\"rx\":0,\"ry\":0,\"a\":false,\"b\":false,\"x\":false,"
                    + "\"y\":false,\"lb\":false,\"rb\":false,\"lt\":0,\"rt\":0,\"dpad\":\"none\"}";

    FakeSimServer(List<String> motorNames) throws IOException {
        this.motorNames = motorNames;
        this.server = new ServerSocket(0);
        this.thread = new Thread(this::serve, "fake-sim");
        this.thread.setDaemon(true);
        this.thread.start();
    }

    int port() {
        return server.getLocalPort();
    }

    void setGamepadJson(String json) {
        this.gamepadJson = json;
    }

    /** Protocol violation simulation: omit the 'ready' state. */
    void setOmitReadyState(boolean omit) {
        this.omitReadyState = omit;
    }

    private void serve() {
        try (Socket s = server.accept();
             BufferedReader in = new BufferedReader(
                     new InputStreamReader(s.getInputStream(), StandardCharsets.UTF_8));
             BufferedWriter out = new BufferedWriter(
                     new OutputStreamWriter(s.getOutputStream(), StandardCharsets.UTF_8))) {

            long tMs = 0;
            double x = 0, y = 0, h = 0;
            double[] ticks = new double[motorNames.size()];

            String line;
            while (running && (line = in.readLine()) != null) {
                Map<String, Object> msg = Json.parseObject(line);
                String type = Json.str(msg, "type");

                if ("reset".equals(type)) {
                    Map<String, Object> pose = Json.obj(msg, "pose");
                    x = Json.num(pose, "x", 0);
                    y = Json.num(pose, "y", 0);
                    h = Json.num(pose, "h", 0);
                    tMs = 0;
                    java.util.Arrays.fill(ticks, 0);
                    StringBuilder sb = new StringBuilder("{\"type\":\"ready\",\"motors\":[");
                    for (int i = 0; i < motorNames.size(); i++) {
                        if (i > 0) {
                            sb.append(',');
                        }
                        sb.append('"').append(motorNames.get(i)).append('"');
                    }
                    sb.append("],\"servos\":[],\"proto\":1");
                    if (!omitReadyState) {
                        sb.append(",\"state\":")
                                .append(state(0, ticks, new double[motorNames.size()], x, y, h));
                    }
                    sb.append('}');
                    write(out, sb.toString());

                } else if ("step".equals(type)) {
                    int dt = (int) Json.num(msg, "dt_ms", 0);
                    // The real server behaves this way; the fake must match it,
                    // or tests pass while integration fails.
                    if (dt <= 0) {
                        throw new SimProtocolException("step.dt_ms must be positive, got " + dt);
                    }
                    Map<String, Object> motors = Json.obj(msg, "motors");

                    double[] p = new double[motorNames.size()];
                    for (int i = 0; i < motorNames.size(); i++) {
                        p[i] = Json.num(motors, motorNames.get(i), 0.0);
                        ticks[i] += p[i] * dt * 0.5;
                    }
                    // Simple mecanum kinematics (assumes fl, fr, bl, br order).
                    double vx = (p[0] + p[1] + p[2] + p[3]) / 4.0;
                    double vy = (-p[0] + p[1] + p[2] - p[3]) / 4.0;
                    double w = (-p[0] + p[1] - p[2] + p[3]) / 4.0;
                    double sec = dt / 1000.0;
                    x += (vx * Math.cos(h) - vy * Math.sin(h)) * 60.0 * sec;
                    y += (vx * Math.sin(h) + vy * Math.cos(h)) * 60.0 * sec;
                    h += w * 3.0 * sec;
                    tMs += dt;

                    write(out, state(tMs, ticks, p, x, y, h));

                } else if ("bye".equals(type)) {
                    return;
                } else {
                    throw new SimProtocolException("fake server does not recognize: " + type);
                }
            }
        } catch (IOException e) {
            // connection closed
        }
    }

    private String state(long tMs, double[] ticks, double[] powers, double x, double y, double h) {
        StringBuilder sb = new StringBuilder(256);
        sb.append("{\"type\":\"state\",\"t_ms\":").append(tMs).append(",\"enc\":{");
        for (int i = 0; i < motorNames.size(); i++) {
            if (i > 0) {
                sb.append(',');
            }
            sb.append('"').append(motorNames.get(i)).append("\":").append((long) ticks[i]);
        }
        sb.append("},\"vel\":{");
        for (int i = 0; i < motorNames.size(); i++) {
            if (i > 0) {
                sb.append(',');
            }
            sb.append('"').append(motorNames.get(i)).append("\":")
                    .append(String.format(Locale.US, "%.3f", powers[i] * 500.0));
        }
        sb.append("},\"imu\":{\"yaw\":").append(String.format(Locale.US, "%.6f", h))
                .append("},\"pinpoint\":{\"x\":").append(String.format(Locale.US, "%.6f", x))
                .append(",\"y\":").append(String.format(Locale.US, "%.6f", y))
                .append(",\"h\":").append(String.format(Locale.US, "%.6f", h))
                .append("},\"voltage\":12.6,\"gamepad\":").append(gamepadJson)
                .append(",\"truth\":{\"x\":").append(String.format(Locale.US, "%.6f", x))
                .append(",\"y\":").append(String.format(Locale.US, "%.6f", y))
                .append(",\"h\":").append(String.format(Locale.US, "%.6f", h))
                .append("}}");
        return sb.toString();
    }

    private static void write(BufferedWriter out, String line) throws IOException {
        out.write(line);
        out.write('\n');
        out.flush();
    }

    @Override
    public void close() throws IOException {
        running = false;
        server.close();
    }
}
