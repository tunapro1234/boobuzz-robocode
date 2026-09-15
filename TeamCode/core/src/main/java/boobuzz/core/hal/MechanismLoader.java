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
        return load(in, "classpath:/mechanism.yaml");
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
            double[] position = requireDoubles(
                    motor.get("pos"), 2, origin, entry.getKey() + ".pos");
            motors.put(entry.getKey(), new Mechanism.Motor(
                    requireStr(motor.get("drives"), origin, entry.getKey() + ".drives"),
                    position[0],
                    position[1],
                    requireNum(motor.get("free_rpm"), origin,
                            entry.getKey() + ".free_rpm")));
            motorNames.add(entry.getKey());
        }
        if (motorNames.isEmpty()) {
            throw new Mechanism.MechanismException(origin + ": en az bir motor tanimlanmali");
        }

        Map<String, Object> drivetrainNode = mapOf(root.get("drivetrain"));
        Mechanism.Drivetrain drivetrain = new Mechanism.Drivetrain(
                requireNum(drivetrainNode.get("wheel_diameter"), origin,
                        "drivetrain.wheel_diameter"));

        Mechanism.Pinpoint pinpoint = pinpoint(root, origin);
        Mechanism.Physics physics = physics(root, origin, motorNames);
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

    private static Mechanism.Physics physics(Map<String, Object> root, String origin,
                                             List<String> motorNames) {
        Map<String, Object> node = mapOf(root.get("physics"));
        Map<String, Object> efficiencyNode = mapOf(node.get("efficiency"));
        Map<String, Double> efficiency = new LinkedHashMap<>();
        for (String name : motorNames) {
            efficiency.put(name, requireNum(
                    efficiencyNode.get(name), origin, "physics.efficiency." + name));
        }
        return new Mechanism.Physics(
                efficiency,
                requireNum(node.get("strafe_eff"), origin, "physics.strafe_eff"),
                requireNum(node.get("zero_power_decel_forward_in_s2"), origin,
                        "physics.zero_power_decel_forward_in_s2"),
                requireNum(node.get("zero_power_decel_lateral_in_s2"), origin,
                        "physics.zero_power_decel_lateral_in_s2"));
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> mapOf(Object node) {
        return node instanceof Map ? (Map<String, Object>) node : Collections.emptyMap();
    }

    private static double requireNum(Object node, String origin, String field) {
        if (!(node instanceof Number number)) {
            throw new Mechanism.MechanismException(
                    origin + ": '" + field + "' sayisal ve zorunlu olmali");
        }
        return number.doubleValue();
    }

    private static String requireStr(Object node, String origin, String field) {
        if (!(node instanceof String value) || value.isBlank()) {
            throw new Mechanism.MechanismException(
                    origin + ": '" + field + "' metin ve zorunlu olmali");
        }
        return value;
    }

    private static double[] requireDoubles(Object node, int size, String origin, String field) {
        if (!(node instanceof List<?> list) || list.size() != size) {
            throw new Mechanism.MechanismException(
                    origin + ": '" + field + "' " + size + " sayi icermeli");
        }
        double[] values = new double[size];
        for (int i = 0; i < values.length; i++) {
            values[i] = requireNum(list.get(i), origin, field + "[" + i + "]");
        }
        return values;
    }
}
