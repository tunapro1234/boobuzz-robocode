package boobuzz.sim;

import boobuzz.core.hal.Mechanism;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.Closeable;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
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
    private final List<String> servoNames;
    private final List<String> encoderNames;
    private final int protocol;
    private volatile boolean running = true;
    // Proto3 only. null readyAnalogs omits ready.analogs; null analog omits state.analog.
    private volatile List<String> readyAnalogs;
    private volatile Map<String, Object> analog;
    private volatile int requestedProto = -1;
    private volatile boolean omitReadyState = false;
    private volatile long readyDelayMs;
    private volatile List<Map<String, Double>> motorFrames = Collections.emptyList();
    private volatile List<Map<String, Double>> servoFrames = Collections.emptyList();
    private volatile String gamepadJson =
            "{\"lx\":0,\"ly\":0,\"rx\":0,\"ry\":0,\"a\":false,\"b\":false,\"x\":false,"
                    + "\"y\":false,\"lb\":false,\"rb\":false,\"lt\":0,\"rt\":0,\"dpad\":\"none\"}";

    /** Legacy proto1 wheel-only fixture. */
    FakeSimServer(List<String> motorNames) throws IOException {
        this(motorNames, Collections.emptyList(), motorNames, 1);
    }

    /** Typed profile fixture for the proto2 seam. */
    FakeSimServer(Mechanism mechanism) throws IOException {
        this(mechanism.motorNames(), mechanism.servoNames(), mechanism.encoderNames(),
                mechanism.usesProto2() ? 2 : 1);
    }

    /**
     * Proto3 typed profile: ready.analogs lists the declared analog inputs. Each state
     * carries no analog value until {@link #setAnalog} provides one.
     */
    FakeSimServer(Mechanism mechanism, int protocol) throws IOException {
        this(mechanism.motorNames(), mechanism.servoNames(), mechanism.encoderNames(), protocol);
        this.readyAnalogs = mechanism.analogInputNames();
    }

    /** Explicit fixture used by mismatch and protocol-negative tests. */
    FakeSimServer(List<String> motorNames, List<String> servoNames,
                  List<String> encoderNames, int protocol) throws IOException {
        if (protocol != 1 && protocol != 2 && protocol != 3) {
            throw new IllegalArgumentException("unsupported fake protocol " + protocol);
        }
        this.motorNames = List.copyOf(motorNames);
        this.servoNames = List.copyOf(servoNames);
        this.encoderNames = List.copyOf(encoderNames);
        this.protocol = protocol;
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

    /** ready.analogs sent on reset; null omits the field. */
    void setReadyAnalogs(List<String> names) {
        this.readyAnalogs = names == null ? null : List.copyOf(names);
    }

    /**
     * state.analog object sent with every state (ready and step). Values may be any
     * JSON number or null, so tests can inject invalid readings; null omits the field.
     */
    void setAnalog(Map<String, Object> values) {
        this.analog = values == null ? null
                : Collections.unmodifiableMap(new LinkedHashMap<>(values));
    }

    /** reset.proto of the last reset; 0 when the field was absent, -1 before any reset. */
    int requestedProto() {
        return requestedProto;
    }

    void setReadyDelayMs(long delayMs) {
        this.readyDelayMs = delayMs;
    }

    /** Copies of the sparse positional-servo maps received from the client. */
    List<Map<String, Double>> servoFrames() {
        return servoFrames;
    }

    /** Copies of the full DC/CR power maps received from the client. */
    List<Map<String, Double>> motorFrames() {
        return motorFrames;
    }

    private void serve() {
        try (Socket s = server.accept();
             BufferedReader in = new BufferedReader(
                     new InputStreamReader(s.getInputStream(), StandardCharsets.UTF_8));
             BufferedWriter out = new BufferedWriter(
                     new OutputStreamWriter(s.getOutputStream(), StandardCharsets.UTF_8))) {

            long tMs = 0;
            double x = 0, y = 0, h = 0;
            double[] ticks = new double[encoderNames.size()];
            List<Map<String, Double>> receivedMotors = new ArrayList<>();
            List<Map<String, Double>> receivedServos = new ArrayList<>();

            String line;
            while (running && (line = in.readLine()) != null) {
                Map<String, Object> msg = Json.parseObject(line);
                String type = Json.str(msg, "type");

                if ("reset".equals(type)) {
                    requestedProto = (int) Json.num(msg, "proto", 0);
                    if (readyDelayMs > 0) {
                        try {
                            Thread.sleep(readyDelayMs);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                            return;
                        }
                    }
                    Map<String, Object> pose = Json.obj(msg, "pose");
                    x = Json.num(pose, "x", 0);
                    y = Json.num(pose, "y", 0);
                    h = Json.num(pose, "h", 0);
                    tMs = 0;
                    Arrays.fill(ticks, 0);
                    receivedMotors.clear();
                    receivedServos.clear();
                    publishMotorFrames(receivedMotors);
                    publishServoFrames(receivedServos);
                    StringBuilder sb = new StringBuilder("{\"type\":\"ready\",\"motors\":[");
                    appendStrings(sb, motorNames);
                    sb.append("],\"servos\":[");
                    appendStrings(sb, servoNames);
                    sb.append("],\"proto\":").append(protocol);
                    List<String> analogNames = readyAnalogs;
                    if (analogNames != null) {
                        sb.append(",\"analogs\":[");
                        appendStrings(sb, analogNames);
                        sb.append(']');
                    }
                    if (!omitReadyState) {
                        sb.append(",\"state\":")
                                .append(state(0, ticks, Collections.emptyMap(), x, y, h));
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
                    Map<String, Object> motorNode = Json.obj(msg, "motors");
                    Map<String, Double> powers = new LinkedHashMap<>();
                    for (String name : motorNames) {
                        powers.put(name, Json.num(motorNode, name, 0.0));
                    }
                    receivedMotors.add(Collections.unmodifiableMap(new LinkedHashMap<>(powers)));
                    publishMotorFrames(receivedMotors);
                    Map<String, Object> servoNode = Json.obj(msg, "servos");
                    Map<String, Double> frameServos = new LinkedHashMap<>();
                    for (String name : servoNames) {
                        if (servoNode.containsKey(name)) {
                            frameServos.put(name, Json.num(servoNode, name, 0.0));
                        }
                    }
                    receivedServos.add(Collections.unmodifiableMap(frameServos));
                    publishServoFrames(receivedServos);

                    for (int i = 0; i < encoderNames.size(); i++) {
                        ticks[i] += powerForEncoder(encoderNames.get(i), powers) * dt * 0.5;
                    }
                    // Simple mecanum kinematics (assumes fl, fr, bl, br keys).
                    double fl = powers.getOrDefault("fl", 0.0);
                    double fr = powers.getOrDefault("fr", 0.0);
                    double bl = powers.getOrDefault("bl", 0.0);
                    double br = powers.getOrDefault("br", 0.0);
                    double vx = (fl + fr + bl + br) / 4.0;
                    double vy = (-fl + fr + bl - br) / 4.0;
                    double w = (-fl + fr - bl + br) / 4.0;
                    double sec = dt / 1000.0;
                    x += (vx * Math.cos(h) - vy * Math.sin(h)) * 60.0 * sec;
                    y += (vx * Math.sin(h) + vy * Math.cos(h)) * 60.0 * sec;
                    h += w * 3.0 * sec;
                    tMs += dt;

                    write(out, state(tMs, ticks, powers, x, y, h));

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

    private double powerForEncoder(String encoder, Map<String, Double> powers) {
        String motor = switch (encoder) {
            case "leftFront" -> "fl";
            case "rightFront" -> "fr";
            case "leftBack" -> "bl";
            case "rightBack" -> "br";
            default -> encoder;
        };
        return powers.getOrDefault(motor, 0.0);
    }

    private String state(long tMs, double[] ticks, Map<String, Double> powers,
                         double x, double y, double h) {
        StringBuilder sb = new StringBuilder(512);
        sb.append("{\"type\":\"state\",\"t_ms\":").append(tMs).append(",\"enc\":{");
        for (int i = 0; i < encoderNames.size(); i++) {
            if (i > 0) sb.append(',');
            String name = encoderNames.get(i);
            sb.append('"').append(name).append("\":").append((long) ticks[i]);
        }
        sb.append("},\"vel\":{");
        for (int i = 0; i < encoderNames.size(); i++) {
            if (i > 0) sb.append(',');
            String name = encoderNames.get(i);
            sb.append('"').append(name).append("\":")
                    .append(String.format(Locale.US, "%.3f", powerForEncoder(name, powers) * 500.0));
        }
        sb.append("},\"imu\":{\"yaw\":").append(String.format(Locale.US, "%.6f", h))
                .append("},\"pinpoint\":{\"x\":").append(String.format(Locale.US, "%.6f", x))
                .append(",\"y\":").append(String.format(Locale.US, "%.6f", y))
                .append(",\"h\":").append(String.format(Locale.US, "%.6f", h))
                .append("},\"voltage\":12.6,\"gamepad\":").append(gamepadJson);
        Map<String, Object> analogValues = analog;
        if (analogValues != null) {
            sb.append(",\"analog\":{");
            int i = 0;
            for (Map.Entry<String, Object> e : analogValues.entrySet()) {
                if (i++ > 0) sb.append(',');
                sb.append('"').append(Json.escape(e.getKey())).append("\":")
                        .append(e.getValue() == null ? "null" : jsonNumber(e.getValue()));
            }
            sb.append('}');
        }
        sb
                .append(",\"truth\":{\"x\":").append(String.format(Locale.US, "%.6f", x))
                .append(",\"y\":").append(String.format(Locale.US, "%.6f", y))
                .append(",\"h\":").append(String.format(Locale.US, "%.6f", h))
                .append("}}");
        return sb.toString();
    }

    private static String jsonNumber(Object value) {
        if (!(value instanceof Number n)) {
            throw new IllegalArgumentException("analog value must be a number: " + value);
        }
        return Double.toString(n.doubleValue());
    }

    private void publishServoFrames(List<Map<String, Double>> frames) {
        servoFrames = Collections.unmodifiableList(new ArrayList<>(frames));
    }

    private void publishMotorFrames(List<Map<String, Double>> frames) {
        motorFrames = Collections.unmodifiableList(new ArrayList<>(frames));
    }

    private static void appendStrings(StringBuilder out, List<String> values) {
        for (int i = 0; i < values.size(); i++) {
            if (i > 0) out.append(',');
            out.append('"').append(Json.escape(values.get(i))).append('"');
        }
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
