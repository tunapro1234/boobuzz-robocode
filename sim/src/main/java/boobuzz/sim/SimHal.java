package boobuzz.sim;

import boobuzz.core.hal.GamepadState;
import boobuzz.core.hal.Hal;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;
import boobuzz.core.hal.Mechanism;

import com.pedropathing.math.Pose;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.Closeable;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * L1'in sim tarafi. TCP istemcisi, satir sonlu JSON, lockstep.
 *
 * <p>Saat sahibi Java'dir: {@code dt_ms}'i biz gondeririz, boylece headless kosu
 * gercek zamandan hizli olabilir. {@link #now()} sunucunun bildirdigi sim saatidir
 * (anayasa kural 2), duvar saati degil.
 *
 * <p>Akis: {@code write()} bir {@code step} gonderir ve karsiligindaki {@code state}'i
 * bloklayarak okur; {@code read()} en son gelen {@code state}'i dondurur. Baglanirken
 * {@code dt_ms=0} ve sifir guclu bir hazirlik adimi atilir, boylece ilk {@code read()}
 * gecerli bir duruma sahiptir ve fizik hic ilerlememis olur.
 *
 * <p>{@code truth} alani okunur ama {@link RobotState}'e KONULMAZ - sadece
 * testler ve viewer icindir. :core gercegi hic gormez.
 */
public final class SimHal implements Hal, Closeable {

    public static final String DEFAULT_HOST = "127.0.0.1";
    public static final int DEFAULT_PORT = 5555;
    private static final int PROTO = 1;

    private final Mechanism mechanism;
    private final Socket socket;
    private final BufferedReader in;
    private final BufferedWriter out;
    private final int dtMs;

    private RobotState state;
    private GamepadState gamepad = GamepadState.neutral();
    private Pose truth;
    private boolean closed;

    public SimHal(Mechanism mechanism, String host, int port, int dtMs,
                  long seed, Pose startPose, int connectTimeoutMs) throws IOException {
        this.mechanism = mechanism;
        this.dtMs = dtMs;
        this.socket = new Socket();
        this.socket.connect(new InetSocketAddress(host, port), connectTimeoutMs);
        this.socket.setTcpNoDelay(true);
        this.in = new BufferedReader(
                new InputStreamReader(socket.getInputStream(), StandardCharsets.UTF_8));
        this.out = new BufferedWriter(
                new OutputStreamWriter(socket.getOutputStream(), StandardCharsets.UTF_8));

        handshake(seed, startPose);
    }

    // ------------------------------------------------------------- el sikisma

    private void handshake(long seed, Pose startPose) throws IOException {
        Pose p = (startPose == null) ? new Pose(0, 0, 0) : startPose;
        StringBuilder sb = new StringBuilder();
        sb.append("{\"type\":\"reset\",\"seed\":").append(seed)
                .append(",\"pose\":{\"x\":").append(fmt(p.x()))
                .append(",\"y\":").append(fmt(p.y()))
                .append(",\"h\":").append(fmt(p.heading()))
                .append("}}");
        send(sb.toString());

        Map<String, Object> ready = receive();
        String type = Json.str(ready, "type");
        if (!"ready".equals(type)) {
            throw new SimProtocolException("'ready' bekleniyordu, '" + type + "' geldi");
        }
        int proto = (int) Json.num(ready, "proto", -1);
        if (proto != PROTO) {
            throw new SimProtocolException(
                    "protokol surumu uyusmuyor: bizde " + PROTO + ", sunucuda " + proto);
        }
        List<String> motors = Json.strings(ready, "motors");
        List<String> servos = Json.strings(ready, "servos");
        // Uyusmazlik = aninda cokme. Yanlis motora guc vermek sahada fark edilmez.
        mechanism.requireNames(motors, servos);

        // 'ready' baslangic state'ini tasir (t_ms = 0, reset pozu).
        Map<String, Object> initial = Json.obj(ready, "state");
        if (initial.isEmpty()) {
            throw new SimProtocolException(
                    "'ready' baslangic 'state' alanini tasimiyor (docs/protokol.md)");
        }
        applyState(initial);
        if (state.t() != 0L) {
            throw new SimProtocolException(
                    "baslangic state t_ms = 0 olmali, " + state.t() + " geldi");
        }
    }

    // ------------------------------------------------------------------- HAL

    @Override
    public long now() {
        return (state == null) ? 0L : state.t();
    }

    @Override
    public RobotState read() {
        if (state == null) {
            throw new SimProtocolException("henuz durum alinmadi (el sikisma tamamlanmadi)");
        }
        return state;
    }

    @Override
    public void write(RobotAction action) {
        try {
            exchange(action, dtMs);
        } catch (IOException e) {
            throw new SimProtocolException("step/state degisimi basarisiz", e);
        }
    }

    @Override
    public GamepadState get() {
        return gamepad;
    }

    // -------------------------------------------------------------- degisim

    private void exchange(RobotAction action, int stepMs) throws IOException {
        StringBuilder sb = new StringBuilder(256);
        sb.append("{\"type\":\"step\",\"dt_ms\":").append(stepMs).append(",\"motors\":");
        Json.writeNumberMap(sb, fill(action.motors(), mechanism.motorNames()));
        sb.append(",\"servos\":");
        Json.writeNumberMap(sb, fill(action.servos(), mechanism.servoNames()));
        sb.append('}');
        send(sb.toString());

        Map<String, Object> msg = receive();
        String type = Json.str(msg, "type");
        if (!"state".equals(type)) {
            throw new SimProtocolException("'state' bekleniyordu, '" + type + "' geldi");
        }
        applyState(msg);
    }

    /** Eksik anahtar = 0; protokol boyle diyor ve sunucuya tam liste gondermek daha guvenli. */
    private static Map<String, Double> fill(Map<String, Double> given, List<String> names) {
        Map<String, Double> out = new LinkedHashMap<>(names.size());
        for (String n : names) {
            out.put(n, given.getOrDefault(n, 0.0));
        }
        return out;
    }

    private void applyState(Map<String, Object> msg) {
        long t = (long) Json.num(msg, "t_ms", 0);

        Map<String, Object> encNode = Json.obj(msg, "enc");
        Map<String, Integer> enc = new HashMap<>(encNode.size());
        for (String name : mechanism.motorNames()) {
            enc.put(name, (int) Math.round(Json.num(encNode, name, 0.0)));
        }

        Map<String, Object> velNode = Json.obj(msg, "vel");
        Map<String, Double> vel = new HashMap<>(velNode.size());
        for (String name : mechanism.motorNames()) {
            vel.put(name, Json.num(velNode, name, 0.0));
        }

        double yaw = Json.num(Json.obj(msg, "imu"), "yaw", 0.0);

        Map<String, Object> pp = Json.obj(msg, "pinpoint");
        Pose pinpoint = new Pose(
                Json.num(pp, "x", 0.0), Json.num(pp, "y", 0.0), Json.num(pp, "h", 0.0));

        double voltage = Json.num(msg, "voltage", 12.6);

        this.state = new RobotState(t, enc, vel, yaw, pinpoint, voltage);
        this.gamepad = readGamepad(Json.obj(msg, "gamepad"));

        Map<String, Object> truthNode = Json.obj(msg, "truth");
        this.truth = truthNode.isEmpty() ? null : new Pose(
                Json.num(truthNode, "x", 0.0),
                Json.num(truthNode, "y", 0.0),
                Json.num(truthNode, "h", 0.0));
    }

    private static GamepadState readGamepad(Map<String, Object> g) {
        if (g.isEmpty()) {
            return GamepadState.neutral();
        }
        String dpad = Json.str(g, "dpad");
        GamepadState.Dpad d;
        try {
            d = (dpad == null || dpad.isBlank() || "none".equalsIgnoreCase(dpad))
                    ? GamepadState.Dpad.NONE
                    : GamepadState.Dpad.valueOf(dpad.trim().toUpperCase(java.util.Locale.ROOT));
        } catch (IllegalArgumentException e) {
            throw new SimProtocolException("bilinmeyen dpad degeri: " + dpad);
        }
        return new GamepadState(
                Json.num(g, "lx", 0), Json.num(g, "ly", 0),
                Json.num(g, "rx", 0), Json.num(g, "ry", 0),
                Json.bool(g, "a"), Json.bool(g, "b"), Json.bool(g, "x"), Json.bool(g, "y"),
                Json.bool(g, "lb"), Json.bool(g, "rb"),
                Json.num(g, "lt", 0), Json.num(g, "rt", 0),
                d);
    }

    // ----------------------------------------------------------------- altyapi

    private void send(String line) throws IOException {
        out.write(line);
        out.write('\n');
        out.flush();
    }

    private Map<String, Object> receive() throws IOException {
        String line = in.readLine();
        if (line == null) {
            throw new ServerClosedException("sunucu baglantiyi kapatti");
        }
        return Json.parseObject(line);
    }

    private static String fmt(double v) {
        return String.format(java.util.Locale.US, "%.6f", v);
    }

    /**
     * Sim gercegi. YALNIZCA testler ve viewer icindir; :core bunu gormez.
     * Sunucu gondermediyse null.
     */
    public Pose truth() {
        return truth;
    }

    @Override
    public void close() throws IOException {
        if (closed) {
            return;
        }
        closed = true;
        try {
            send("{\"type\":\"bye\"}");
        } catch (IOException ignored) {
            // kapanirken yazamamak sorun degil
        } finally {
            socket.close();
        }
    }
}
