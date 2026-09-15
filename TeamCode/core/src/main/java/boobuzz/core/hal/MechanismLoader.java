package boobuzz.core.hal;

import org.yaml.snakeyaml.Yaml;

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Reader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** {@code mechanism.yaml} dosyasini saf {@link Mechanism} verisine donusturur. */
public final class MechanismLoader {

    private MechanismLoader() {}

    public static Mechanism load(Path path) throws IOException {
        try (Reader reader = Files.newBufferedReader(path, StandardCharsets.UTF_8)) {
            return parse(reader, path.toString());
        }
    }

    public static Mechanism load(InputStream in, String origin) {
        return parse(new InputStreamReader(in, StandardCharsets.UTF_8), origin);
    }

    /** Core jar'a paketlenen ortak robot mekanizmasini yukler. */
    public static Mechanism loadDefault() {
        InputStream in = MechanismLoader.class.getResourceAsStream("/mechanism.yaml");
        if (in == null) {
            throw new Mechanism.MechanismException("classpath'te mechanism.yaml bulunamadi");
        }
        try (in) {
            return load(in, "classpath:/mechanism.yaml");
        } catch (IOException e) {
            throw new Mechanism.MechanismException(
                    "mechanism.yaml kapatilamadi: " + e.getMessage());
        }
    }

    @SuppressWarnings("unchecked")
    private static Mechanism parse(Reader reader, String origin) {
        Object loaded = new Yaml().load(reader);
        if (!(loaded instanceof Map)) {
            throw new Mechanism.MechanismException(origin + ": kok bir esleme (map) olmali");
        }
        Map<String, Object> root = (Map<String, Object>) loaded;

        Map<String, Object> motorsNode = mapOf(root.get("motors"));
        Map<String, Mechanism.Motor> motors = new LinkedHashMap<>();
        List<String> motorNames = new ArrayList<>();
        for (Map.Entry<String, Object> entry : motorsNode.entrySet()) {
            Map<String, Object> motor = mapOf(entry.getValue());
            double[] position = doubles(motor.get("pos"), new double[] {0.0, 0.0});
            motors.put(entry.getKey(), new Mechanism.Motor(
                    str(motor.get("drives"), "unknown"),
                    coordinate(position, 0),
                    coordinate(position, 1),
                    requireNum(motor.get("free_rpm"), origin,
                            entry.getKey() + ".free_rpm")));
            motorNames.add(entry.getKey());
        }
        if (motorNames.isEmpty()) {
            throw new Mechanism.MechanismException(origin + ": en az bir motor tanimlanmali");
        }

        Map<String, Object> drivetrainNode = mapOf(root.get("drivetrain"));
        Mechanism.Drivetrain drivetrain = new Mechanism.Drivetrain(
                num(drivetrainNode.get("wheel_diameter"), 0.0));

        Mechanism.Pinpoint pinpoint = pinpoint(root, origin);
        Mechanism.Physics physics = physics(root);
        return new Mechanism(motorNames, new ArrayList<>(mapOf(root.get("servos")).keySet()),
                motors, drivetrain, pinpoint, physics);
    }

    private static Mechanism.Pinpoint pinpoint(Map<String, Object> root, String origin) {
        Map<String, Object> sensors = mapOf(root.get("sensors"));
        if (!sensors.containsKey("pinpoint")) {
            return null;
        }
        Map<String, Object> node = mapOf(sensors.get("pinpoint"));
        return new Mechanism.Pinpoint(
                requireNum(node.get("x_pod_offset_mm"), origin,
                        "sensors.pinpoint.x_pod_offset_mm"),
                requireNum(node.get("y_pod_offset_mm"), origin,
                        "sensors.pinpoint.y_pod_offset_mm"),
                requireStr(node.get("x_pod_direction"), origin,
                        "sensors.pinpoint.x_pod_direction"),
                requireStr(node.get("y_pod_direction"), origin,
                        "sensors.pinpoint.y_pod_direction"),
                requireStr(node.get("pod_type"), origin, "sensors.pinpoint.pod_type"));
    }

    private static Mechanism.Physics physics(Map<String, Object> root) {
        Map<String, Object> node = mapOf(root.get("physics"));
        Map<String, Double> efficiency = new LinkedHashMap<>();
        for (Map.Entry<String, Object> entry : mapOf(node.get("efficiency")).entrySet()) {
            efficiency.put(entry.getKey(), num(entry.getValue(), 1.0));
        }
        return new Mechanism.Physics(
                efficiency,
                num(node.get("strafe_eff"), 0.0),
                num(node.get("zero_power_decel_forward_in_s2"), 0.0),
                num(node.get("zero_power_decel_lateral_in_s2"), 0.0));
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> mapOf(Object node) {
        return node instanceof Map ? (Map<String, Object>) node : Collections.emptyMap();
    }

    private static String str(Object node, String fallback) {
        return node == null ? fallback : String.valueOf(node);
    }

    private static double num(Object node, double fallback) {
        if (node instanceof Number number) {
            return number.doubleValue();
        }
        if (node instanceof String text && !text.isBlank()) {
            return Double.parseDouble(text.trim());
        }
        return fallback;
    }

    private static double requireNum(Object node, String origin, String field) {
        if (node == null) {
            throw new Mechanism.MechanismException(
                    origin + ": '" + field + "' zorunlu alan, eksik");
        }
        return num(node, 0.0);
    }

    private static String requireStr(Object node, String origin, String field) {
        String value = str(node, null);
        if (value == null || value.isBlank()) {
            throw new Mechanism.MechanismException(
                    origin + ": '" + field + "' zorunlu alan, eksik");
        }
        return value;
    }

    private static double[] doubles(Object node, double[] fallback) {
        if (!(node instanceof List<?> list)) {
            return fallback;
        }
        double[] values = new double[list.size()];
        for (int i = 0; i < values.length; i++) {
            values[i] = num(list.get(i), 0.0);
        }
        return values;
    }

    private static double coordinate(double[] position, int index) {
        return position.length > index ? position[index] : 0.0;
    }
}
