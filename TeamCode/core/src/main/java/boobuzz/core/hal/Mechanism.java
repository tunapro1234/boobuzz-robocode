package boobuzz.core.hal;

import org.yaml.snakeyaml.Yaml;

import java.io.IOException;
import java.io.InputStream;
import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * {@code mechanism.yaml} okuyucusu.
 *
 * <p>Tek gercek kaynak: sim geometrisi VE gercek robotun koordinat matematigi ayni
 * dosyadan gelir. Motor/servo ADLARI buradan gelir, protokol semasi sabit kalir.
 *
 * <p>Dosyada aci DERECE, protokolde RADYAN. Donusum burada yapilir; disari
 * cikan her aci radyandir.
 */
public final class Mechanism {

    private final List<String> motorNames;
    private final List<String> servoNames;
    private final Map<String, Motor> motors;
    private final Map<String, Servo> servos;
    private final Map<String, Frame> frames;
    private final Map<String, Sensor> sensors;
    private final Drivetrain drivetrain;
    private final Footprint robot;
    private final Pinpoint pinpoint;

    private Mechanism(List<String> motorNames, List<String> servoNames,
                      Map<String, Motor> motors, Map<String, Servo> servos,
                      Map<String, Frame> frames, Map<String, Sensor> sensors,
                      Drivetrain drivetrain, Footprint robot, Pinpoint pinpoint) {
        this.motorNames = List.copyOf(motorNames);
        this.servoNames = List.copyOf(servoNames);
        this.motors = Map.copyOf(motors);
        this.servos = Map.copyOf(servos);
        this.frames = Map.copyOf(frames);
        this.sensors = Map.copyOf(sensors);
        this.drivetrain = drivetrain;
        this.robot = robot;
        this.pinpoint = pinpoint;
    }

    // ---------------------------------------------------------------- yukleme

    public static Mechanism load(Path path) throws IOException {
        try (Reader reader = Files.newBufferedReader(path, StandardCharsets.UTF_8)) {
            return parse(reader, path.toString());
        }
    }

    public static Mechanism load(InputStream in, String origin) {
        return parse(new java.io.InputStreamReader(in, StandardCharsets.UTF_8), origin);
    }

    /** Core jar'a root kaynaktan paketlenen ortak robot mekanizmasini yukler. */
    public static Mechanism loadDefault() {
        InputStream in = Mechanism.class.getResourceAsStream("/mechanism.yaml");
        if (in == null) {
            throw new MechanismException("classpath'te mechanism.yaml bulunamadi");
        }
        try (in) {
            return load(in, "classpath:/mechanism.yaml");
        } catch (IOException e) {
            throw new MechanismException("mechanism.yaml kapatilamadi: " + e.getMessage());
        }
    }

    @SuppressWarnings("unchecked")
    private static Mechanism parse(Reader reader, String origin) {
        Object loaded = new Yaml().load(reader);
        if (!(loaded instanceof Map)) {
            throw new MechanismException(origin + ": kok bir esleme (map) olmali");
        }
        Map<String, Object> root = (Map<String, Object>) loaded;

        Map<String, Object> unitsNode = mapOf(root.get("units"));
        String angleUnit = str(unitsNode.get("angle"), "deg");
        boolean degrees = !"rad".equalsIgnoreCase(angleUnit);

        // --- motorlar
        Map<String, Object> motorsNode = mapOf(root.get("motors"));
        Map<String, Motor> motors = new LinkedHashMap<>();
        List<String> motorNames = new ArrayList<>();
        for (Map.Entry<String, Object> e : motorsNode.entrySet()) {
            Map<String, Object> m = mapOf(e.getValue());
            double[] pos = doubles(m.get("pos"), new double[] {0, 0});
            motors.put(e.getKey(), new Motor(
                    e.getKey(),
                    str(m.get("drives"), "unknown"),
                    pos,
                    angle(num(m.get("roller"), 0.0), degrees),
                    num(m.get("ticks_per_rev"), 0.0),
                    requireNum(m.get("free_rpm"), origin, e.getKey() + ".free_rpm"),
                    num(m.get("kV"), 0.0),
                    num(m.get("kS"), 0.0),
                    num(m.get("gear"), 1.0)));
            motorNames.add(e.getKey());
        }

        // --- servolar
        Map<String, Object> servosNode = mapOf(root.get("servos"));
        Map<String, Servo> servos = new LinkedHashMap<>();
        List<String> servoNames = new ArrayList<>();
        for (Map.Entry<String, Object> e : servosNode.entrySet()) {
            Map<String, Object> s = mapOf(e.getValue());
            servos.put(e.getKey(), new Servo(
                    e.getKey(),
                    str(s.get("drives"), "unknown"),
                    angle(num(s.get("min"), 0.0), degrees),
                    angle(num(s.get("max"), 0.0), degrees)));
            servoNames.add(e.getKey());
        }

        // --- tf agaci
        Map<String, Object> framesNode = mapOf(root.get("frames"));
        Map<String, Frame> frames = new LinkedHashMap<>();
        for (Map.Entry<String, Object> e : framesNode.entrySet()) {
            Map<String, Object> f = mapOf(e.getValue());
            double[] rpyDeg = doubles(f.get("rpy"), new double[] {0, 0, 0});
            double[] rpy = new double[] {
                    angle(rpyDeg[0], degrees), angle(rpyDeg[1], degrees), angle(rpyDeg[2], degrees)};
            double[] limDeg = doubles(f.get("limits"), null);
            double[] limits = (limDeg == null) ? null
                    : new double[] {angle(limDeg[0], degrees), angle(limDeg[1], degrees)};
            frames.put(e.getKey(), new Frame(
                    e.getKey(),
                    str(f.get("parent"), null),
                    doubles(f.get("xyz"), new double[] {0, 0, 0}),
                    rpy,
                    str(f.get("joint"), "fixed"),
                    str(f.get("axis"), null),
                    limits,
                    angle(num(f.get("hfov"), 0.0), degrees),
                    angle(num(f.get("vfov"), 0.0), degrees)));
        }

        // --- sensorler
        Map<String, Object> sensorsNode = mapOf(root.get("sensors"));
        Map<String, Sensor> sensors = new LinkedHashMap<>();
        for (Map.Entry<String, Object> e : sensorsNode.entrySet()) {
            Map<String, Object> s = mapOf(e.getValue());
            sensors.put(e.getKey(), new Sensor(
                    e.getKey(),
                    str(s.get("parent"), "robot"),
                    doubles(s.get("xyz"), new double[] {0, 0, 0})));
        }
        Pinpoint pinpoint = null;
        if (sensorsNode.containsKey("pinpoint")) {
            Map<String, Object> p = mapOf(sensorsNode.get("pinpoint"));
            pinpoint = new Pinpoint(
                    requireNum(p.get("x_pod_offset_mm"), origin,
                            "sensors.pinpoint.x_pod_offset_mm"),
                    requireNum(p.get("y_pod_offset_mm"), origin,
                            "sensors.pinpoint.y_pod_offset_mm"),
                    requireStr(p.get("x_pod_direction"), origin,
                            "sensors.pinpoint.x_pod_direction"),
                    requireStr(p.get("y_pod_direction"), origin,
                            "sensors.pinpoint.y_pod_direction"),
                    requireStr(p.get("pod_type"), origin, "sensors.pinpoint.pod_type"));
        }

        // --- drivetrain
        Map<String, Object> dt = mapOf(root.get("drivetrain"));
        Drivetrain drivetrain = new Drivetrain(
                str(dt.get("type"), "mecanum"),
                num(dt.get("track_width"), 0.0),
                num(dt.get("wheel_base"), 0.0),
                num(dt.get("wheel_diameter"), 0.0));

        if (motorNames.isEmpty()) {
            throw new MechanismException(origin + ": en az bir motor tanimlanmali");
        }
        validateFrames(frames, origin);

        Map<String, Object> robotNode = mapOf(root.get("robot"));
        Footprint robot = new Footprint(
                num(robotNode.get("width"), 0.0), num(robotNode.get("length"), 0.0));

        return new Mechanism(motorNames, servoNames, motors, servos, frames, sensors,
                drivetrain, robot, pinpoint);
    }

    private static void validateFrames(Map<String, Frame> frames, String origin) {
        for (Frame f : frames.values()) {
            String parent = f.parent();
            if (parent == null || "field".equals(parent)) {
                continue;
            }
            if (!frames.containsKey(parent)) {
                throw new MechanismException(
                        origin + ": '" + f.name() + "' cercevesinin ebeveyni '" + parent + "' tanimli degil");
            }
        }
    }

    // ---------------------------------------------------------------- dogrulama

    /**
     * Sunucunun bildirdigi ad listesiyle karsilastirir. Uyusmazlik = aninda cokme.
     * Sessiz kayma yasak: yanlis motora guc vermek sahada fark edilmez.
     */
    public void requireNames(List<String> actualMotors, List<String> actualServos) {
        List<String> expectedM = new ArrayList<>(motorNames);
        List<String> gotM = new ArrayList<>(actualMotors == null ? List.of() : actualMotors);
        List<String> expectedS = new ArrayList<>(servoNames);
        List<String> gotS = new ArrayList<>(actualServos == null ? List.of() : actualServos);
        Collections.sort(expectedM);
        Collections.sort(gotM);
        Collections.sort(expectedS);
        Collections.sort(gotS);
        if (!expectedM.equals(gotM) || !expectedS.equals(gotS)) {
            throw new MechanismException(
                    "mechanism.yaml ile sunucu ad listesi uyusmuyor.\n"
                            + "  motor  beklenen=" + expectedM + " gelen=" + gotM + "\n"
                            + "  servo  beklenen=" + expectedS + " gelen=" + gotS);
        }
    }

    // ---------------------------------------------------------------- erisim

    public List<String> motorNames() { return motorNames; }

    public List<String> servoNames() { return servoNames; }

    public Map<String, Motor> motors() { return motors; }

    public Map<String, Servo> servos() { return servos; }

    public Map<String, Frame> frames() { return frames; }

    public Map<String, Sensor> sensors() { return sensors; }

    public Drivetrain drivetrain() { return drivetrain; }

    public Pinpoint pinpoint() {
        if (pinpoint == null) {
            throw new MechanismException("mechanism.yaml'da sensors.pinpoint yok");
        }
        return pinpoint;
    }

    /** Ayak izi; duvar kirpma ve cizim icin. */
    public Footprint robot() { return robot; }

    public Motor motor(String name) {
        Motor m = motors.get(name);
        if (m == null) {
            throw new MechanismException("mechanism.yaml'da '" + name + "' motoru yok");
        }
        return m;
    }

    /** {@code drives: wheel} olan motorlar; C1 bunlari surer. */
    public List<String> wheelMotorNames() {
        List<String> out = new ArrayList<>();
        for (String n : motorNames) {
            if ("wheel".equals(motors.get(n).drives())) {
                out.add(n);
            }
        }
        return out;
    }

    // ---------------------------------------------------------------- tipler

    public record Motor(String name, String drives, double[] pos, double rollerRad,
                        double ticksPerRev, double freeRpm, double kV, double kS, double gear) {

        /** pos = [x ileri, y sol] (docs/protokol.md cerceve kurali). */
        public double forward() { return pos[0]; }

        public double left() { return pos[1]; }
    }

    public record Servo(String name, String drives, double minRad, double maxRad) {}

    public record Frame(String name, String parent, double[] xyz, double[] rpyRad,
                        String joint, String axis, double[] limitsRad,
                        double hfovRad, double vfovRad) {}

    public record Sensor(String name, String parent, double[] xyz) {}

    public record Drivetrain(String type, double trackWidth, double wheelBase, double wheelDiameter) {}

    public record Pinpoint(double xPodOffsetMm, double yPodOffsetMm,
                           String xPodDirection, String yPodDirection, String podType) {}

    public record Footprint(double width, double length) {}

    public static final class MechanismException extends RuntimeException {
        public MechanismException(String message) {
            super(message);
        }
    }

    // ---------------------------------------------------------------- yardimci

    @SuppressWarnings("unchecked")
    private static Map<String, Object> mapOf(Object node) {
        if (node instanceof Map) {
            return (Map<String, Object>) node;
        }
        return Collections.emptyMap();
    }

    private static String str(Object node, String fallback) {
        return (node == null) ? fallback : String.valueOf(node);
    }

    private static double num(Object node, double fallback) {
        if (node instanceof Number n) {
            return n.doubleValue();
        }
        if (node instanceof String s && !s.isBlank()) {
            return Double.parseDouble(s.trim());
        }
        return fallback;
    }

    private static double[] doubles(Object node, double[] fallback) {
        if (!(node instanceof List<?> list)) {
            return fallback;
        }
        double[] out = new double[list.size()];
        for (int i = 0; i < out.length; i++) {
            out[i] = num(list.get(i), 0.0);
        }
        return out;
    }

    private static double requireNum(Object node, String origin, String what) {
        if (node == null) {
            throw new MechanismException(origin + ": '" + what + "' zorunlu alan, eksik");
        }
        return num(node, 0.0);
    }

    private static String requireStr(Object node, String origin, String what) {
        String value = str(node, null);
        if (value == null || value.isBlank()) {
            throw new MechanismException(origin + ": '" + what + "' zorunlu alan, eksik");
        }
        return value;
    }

    private static double angle(double value, boolean degrees) {
        return degrees ? Math.toRadians(value) : value;
    }
}
