package boobuzz.sim;

import org.yaml.snakeyaml.Yaml;

import java.util.List;
import java.util.Map;

/**
 * Satir sonlu JSON. Okuma icin snakeyaml (JSON, YAML'in alt kumesidir),
 * yazma icin elle - protokolde sadece duz nesneler var.
 */
final class Json {

    private Json() {}

    @SuppressWarnings("unchecked")
    static Map<String, Object> parseObject(String line) {
        Object parsed = new Yaml().load(line);
        if (!(parsed instanceof Map)) {
            throw new SimProtocolException("JSON nesnesi bekleniyordu: " + trim(line));
        }
        return (Map<String, Object>) parsed;
    }

    static String str(Map<String, Object> node, String key) {
        Object v = node.get(key);
        return (v == null) ? null : String.valueOf(v);
    }

    static double num(Map<String, Object> node, String key, double fallback) {
        Object v = node.get(key);
        if (v instanceof Number n) {
            return n.doubleValue();
        }
        return fallback;
    }

    static boolean bool(Map<String, Object> node, String key) {
        Object v = node.get(key);
        if (v instanceof Boolean b) {
            return b;
        }
        return v instanceof String s && "true".equalsIgnoreCase(s.trim());
    }

    @SuppressWarnings("unchecked")
    static Map<String, Object> obj(Map<String, Object> node, String key) {
        Object v = node.get(key);
        return (v instanceof Map) ? (Map<String, Object>) v : Map.of();
    }

    @SuppressWarnings("unchecked")
    static List<String> strings(Map<String, Object> node, String key) {
        Object v = node.get(key);
        if (!(v instanceof List)) {
            return List.of();
        }
        return ((List<Object>) v).stream().map(String::valueOf).toList();
    }

    /** {@code {"a":1.0,"b":2.0}} — protokolde motor/servo haritalari icin. */
    static void writeNumberMap(StringBuilder out, Map<String, Double> values) {
        out.append('{');
        boolean first = true;
        for (Map.Entry<String, Double> e : values.entrySet()) {
            if (!first) {
                out.append(',');
            }
            first = false;
            out.append('"').append(escape(e.getKey())).append("\":");
            double v = e.getValue();
            if (!Double.isFinite(v)) {
                throw new SimProtocolException("sonlu olmayan deger: " + e.getKey() + "=" + v);
            }
            out.append(String.format(java.util.Locale.US, "%.6f", v));
        }
        out.append('}');
    }

    static String escape(String s) {
        return s.replace("\\", "\\\\").replace("\"", "\\\"");
    }

    private static String trim(String line) {
        return line.length() > 200 ? line.substring(0, 200) + "..." : line;
    }
}
