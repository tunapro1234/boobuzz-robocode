package boobuzz.sim;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Minimal line-oriented JSON reader/writer for the simulator protocol. */
final class Json {

    private Json() {}

    static Map<String, Object> parseObject(String line) {
        if (line == null) {
            throw new SimProtocolException("JSON line is null");
        }
        Parser parser = new Parser(line);
        Object parsed = parser.value();
        parser.whitespace();
        if (!parser.end()) {
            throw parser.error("trailing characters");
        }
        if (!(parsed instanceof Map<?, ?>)) {
            throw new SimProtocolException("JSON object expected: " + trim(line));
        }
        @SuppressWarnings("unchecked")
        Map<String, Object> object = (Map<String, Object>) parsed;
        return object;
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
        List<String> result = new ArrayList<>();
        for (Object item : (List<Object>) v) {
            result.add(String.valueOf(item));
        }
        return result;
    }

    /** {@code {"a":1.0,"b":2.0}} — for motor/servo maps in the protocol. */
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
                throw new SimProtocolException("non-finite value: " + e.getKey() + "=" + v);
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

    private static final class Parser {
        private final String input;
        private int index;

        Parser(String input) {
            this.input = input;
        }

        Object value() {
            whitespace();
            if (end()) {
                throw error("value expected");
            }
            return switch (input.charAt(index)) {
                case '{' -> object();
                case '[' -> array();
                case '"' -> string();
                case 't' -> literal("true", Boolean.TRUE);
                case 'f' -> literal("false", Boolean.FALSE);
                case 'n' -> literal("null", null);
                default -> number();
            };
        }

        private Map<String, Object> object() {
            index++;
            Map<String, Object> result = new LinkedHashMap<>();
            whitespace();
            if (consume('}')) {
                return result;
            }
            while (true) {
                whitespace();
                if (end() || input.charAt(index) != '"') {
                    throw error("object key expected");
                }
                String key = string();
                whitespace();
                expect(':');
                result.put(key, value());
                whitespace();
                if (consume('}')) {
                    return result;
                }
                expect(',');
            }
        }

        private List<Object> array() {
            index++;
            List<Object> result = new ArrayList<>();
            whitespace();
            if (consume(']')) {
                return result;
            }
            while (true) {
                result.add(value());
                whitespace();
                if (consume(']')) {
                    return result;
                }
                expect(',');
            }
        }

        private String string() {
            expect('"');
            StringBuilder result = new StringBuilder();
            while (!end()) {
                char c = input.charAt(index++);
                if (c == '"') {
                    return result.toString();
                }
                if (c == '\\') {
                    if (end()) {
                        throw error("escape sequence is incomplete");
                    }
                    char escaped = input.charAt(index++);
                    switch (escaped) {
                        case '"', '\\', '/' -> result.append(escaped);
                        case 'b' -> result.append('\b');
                        case 'f' -> result.append('\f');
                        case 'n' -> result.append('\n');
                        case 'r' -> result.append('\r');
                        case 't' -> result.append('\t');
                        case 'u' -> result.append(unicodeEscape());
                        default -> throw error("invalid escape sequence");
                    }
                } else {
                    if (c < 0x20) {
                        throw error("control character in string");
                    }
                    result.append(c);
                }
            }
            throw error("unterminated string");
        }

        private char unicodeEscape() {
            if (index + 4 > input.length()) {
                throw error("unicode escape is incomplete");
            }
            int value = 0;
            for (int i = 0; i < 4; i++) {
                int digit = Character.digit(input.charAt(index++), 16);
                if (digit < 0) {
                    throw error("invalid unicode escape");
                }
                value = value * 16 + digit;
            }
            return (char) value;
        }

        private Object literal(String expected, Object value) {
            if (!input.startsWith(expected, index)) {
                throw error("invalid literal");
            }
            index += expected.length();
            return value;
        }

        private double number() {
            int start = index;
            consume('-');
            if (end() || !digit(input.charAt(index))) {
                throw error("number expected");
            }
            if (input.charAt(index) == '0') {
                index++;
            } else {
                while (!end() && digit(input.charAt(index))) {
                    index++;
                }
            }
            if (consume('.')) {
                digits();
            }
            if (!end() && (input.charAt(index) == 'e' || input.charAt(index) == 'E')) {
                index++;
                consume('+');
                consume('-');
                digits();
            }
            String token = input.substring(start, index);
            try {
                double value = Double.parseDouble(token);
                if (!Double.isFinite(value)) {
                    throw error("number is not finite");
                }
                return value;
            } catch (NumberFormatException e) {
                throw new SimProtocolException("invalid JSON number: " + token, e);
            }
        }

        private void digits() {
            int start = index;
            while (!end() && digit(input.charAt(index))) {
                index++;
            }
            if (start == index) {
                throw error("digit expected");
            }
        }

        private boolean digit(char c) {
            return c >= '0' && c <= '9';
        }

        private void whitespace() {
            while (!end() && Character.isWhitespace(input.charAt(index))) {
                index++;
            }
        }

        private boolean consume(char expected) {
            if (!end() && input.charAt(index) == expected) {
                index++;
                return true;
            }
            return false;
        }

        private void expect(char expected) {
            if (!consume(expected)) {
                throw error("expected '" + expected + "'");
            }
        }

        private boolean end() {
            return index >= input.length();
        }

        private SimProtocolException error(String message) {
            return new SimProtocolException("invalid JSON at " + index + ": " + message);
        }
    }
}
