package boobuzz.core.debug;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Small dependency-free JSON codec shared by taps, bags, and socket control. */
public final class JsonCodec {

    private JsonCodec() {}

    public static Map<String, Object> parseObject(String line) {
        Object value = new Parser(line).value();
        if (!(value instanceof Map<?, ?> map)) {
            throw new IllegalArgumentException("JSON object expected");
        }
        @SuppressWarnings("unchecked")
        Map<String, Object> result = (Map<String, Object>) map;
        return result;
    }

    public static String stringify(Object value) {
        StringBuilder out = new StringBuilder();
        write(out, value);
        return out.toString();
    }

    public static String str(Map<String, Object> object, String key, String fallback) {
        Object value = object.get(key);
        return value == null ? fallback : String.valueOf(value);
    }

    public static double num(Map<String, Object> object, String key, double fallback) {
        Object value = object.get(key);
        return value instanceof Number number ? number.doubleValue() : fallback;
    }

    public static boolean bool(Map<String, Object> object, String key, boolean fallback) {
        Object value = object.get(key);
        if (value instanceof Boolean bool) {
            return bool;
        }
        return value == null ? fallback : Boolean.parseBoolean(String.valueOf(value));
    }

    @SuppressWarnings("unchecked")
    public static Map<String, Object> object(Map<String, Object> object, String key) {
        Object value = object.get(key);
        return value instanceof Map<?, ?> map ? (Map<String, Object>) map
                : java.util.Collections.emptyMap();
    }

    @SuppressWarnings("unchecked")
    public static List<Object> list(Map<String, Object> object, String key) {
        Object value = object.get(key);
        return value instanceof List<?> list ? (List<Object>) list
                : java.util.Collections.emptyList();
    }

    private static void write(StringBuilder out, Object value) {
        if (value == null) {
            out.append("null");
        } else if (value instanceof String string) {
            out.append('"').append(escape(string)).append('"');
        } else if (value instanceof Character character) {
            out.append('"').append(escape(character.toString())).append('"');
        } else if (value instanceof Number number) {
            double numeric = number.doubleValue();
            if (!Double.isFinite(numeric)) {
                throw new IllegalArgumentException("non-finite JSON number: " + numeric);
            }
            out.append(number instanceof Float || number instanceof Double
                    ? Double.toString(numeric) : number.toString());
        } else if (value instanceof Boolean) {
            out.append(value);
        } else if (value instanceof Map<?, ?> map) {
            out.append('{');
            boolean first = true;
            for (Map.Entry<?, ?> entry : map.entrySet()) {
                if (!first) out.append(',');
                first = false;
                write(out, String.valueOf(entry.getKey()));
                out.append(':');
                write(out, entry.getValue());
            }
            out.append('}');
        } else if (value instanceof Iterable<?> iterable) {
            out.append('[');
            boolean first = true;
            for (Object item : iterable) {
                if (!first) out.append(',');
                first = false;
                write(out, item);
            }
            out.append(']');
        } else if (value.getClass().isArray()) {
            out.append('[');
            int length = java.lang.reflect.Array.getLength(value);
            for (int i = 0; i < length; i++) {
                if (i > 0) out.append(',');
                write(out, java.lang.reflect.Array.get(value, i));
            }
            out.append(']');
        } else {
            throw new IllegalArgumentException("unsupported JSON value: " + value.getClass());
        }
    }

    private static String escape(String value) {
        StringBuilder escaped = new StringBuilder(value.length() + 8);
        for (int i = 0; i < value.length(); i++) {
            char c = value.charAt(i);
            switch (c) {
                case '"' -> escaped.append("\\\"");
                case '\\' -> escaped.append("\\\\");
                case '\b' -> escaped.append("\\b");
                case '\f' -> escaped.append("\\f");
                case '\n' -> escaped.append("\\n");
                case '\r' -> escaped.append("\\r");
                case '\t' -> escaped.append("\\t");
                default -> {
                    if (c < 0x20) {
                        escaped.append(String.format("\\u%04x", (int) c));
                    } else {
                        escaped.append(c);
                    }
                }
            }
        }
        return escaped.toString();
    }

    private static final class Parser {
        private final String input;
        private int index;

        private Parser(String input) {
            this.input = input == null ? "" : input;
        }

        private Object value() {
            whitespace();
            if (end()) throw error("value expected");
            Object result = switch (input.charAt(index)) {
                case '{' -> objectValue();
                case '[' -> arrayValue();
                case '"' -> stringValue();
                case 't' -> literal("true", Boolean.TRUE);
                case 'f' -> literal("false", Boolean.FALSE);
                case 'n' -> literal("null", null);
                default -> numberValue();
            };
            whitespace();
            if (!end()) throw error("trailing characters");
            return result;
        }

        private Map<String, Object> objectValue() {
            index++;
            Map<String, Object> result = new LinkedHashMap<>();
            whitespace();
            if (consume('}')) return result;
            while (true) {
                whitespace();
                if (end() || input.charAt(index) != '"') throw error("object key expected");
                String key = stringValue();
                whitespace();
                expect(':');
                result.put(key, valueWithoutTrailingCheck());
                whitespace();
                if (consume('}')) return result;
                expect(',');
            }
        }

        private List<Object> arrayValue() {
            index++;
            List<Object> result = new ArrayList<>();
            whitespace();
            if (consume(']')) return result;
            while (true) {
                result.add(valueWithoutTrailingCheck());
                whitespace();
                if (consume(']')) return result;
                expect(',');
            }
        }

        private Object valueWithoutTrailingCheck() {
            whitespace();
            if (end()) throw error("value expected");
            return switch (input.charAt(index)) {
                case '{' -> objectValue();
                case '[' -> arrayValue();
                case '"' -> stringValue();
                case 't' -> literal("true", Boolean.TRUE);
                case 'f' -> literal("false", Boolean.FALSE);
                case 'n' -> literal("null", null);
                default -> numberValue();
            };
        }

        private String stringValue() {
            expect('"');
            StringBuilder result = new StringBuilder();
            while (!end()) {
                char c = input.charAt(index++);
                if (c == '"') return result.toString();
                if (c == '\\') {
                    if (end()) throw error("incomplete escape");
                    char escaped = input.charAt(index++);
                    switch (escaped) {
                        case '"', '\\', '/' -> result.append(escaped);
                        case 'b' -> result.append('\b');
                        case 'f' -> result.append('\f');
                        case 'n' -> result.append('\n');
                        case 'r' -> result.append('\r');
                        case 't' -> result.append('\t');
                        case 'u' -> result.append(unicode());
                        default -> throw error("invalid escape");
                    }
                } else {
                    if (c < 0x20) throw error("control character");
                    result.append(c);
                }
            }
            throw error("unterminated string");
        }

        private char unicode() {
            if (index + 4 > input.length()) throw error("incomplete unicode escape");
            int value = 0;
            for (int i = 0; i < 4; i++) {
                int digit = Character.digit(input.charAt(index++), 16);
                if (digit < 0) throw error("invalid unicode escape");
                value = value * 16 + digit;
            }
            return (char) value;
        }

        private Object literal(String expected, Object value) {
            if (!input.startsWith(expected, index)) throw error("invalid literal");
            index += expected.length();
            return value;
        }

        private Double numberValue() {
            int start = index;
            consume('-');
            if (end() || !digit(input.charAt(index))) throw error("number expected");
            if (input.charAt(index) == '0') index++;
            else while (!end() && digit(input.charAt(index))) index++;
            if (consume('.')) digits();
            if (!end() && (input.charAt(index) == 'e' || input.charAt(index) == 'E')) {
                index++;
                consume('+');
                consume('-');
                digits();
            }
            String token = input.substring(start, index);
            try {
                double value = Double.parseDouble(token);
                if (!Double.isFinite(value)) throw error("non-finite number");
                return value;
            } catch (NumberFormatException e) {
                throw error("invalid number");
            }
        }

        private void digits() {
            int start = index;
            while (!end() && digit(input.charAt(index))) index++;
            if (start == index) throw error("digit expected");
        }

        private boolean digit(char c) { return c >= '0' && c <= '9'; }
        private void whitespace() { while (!end() && Character.isWhitespace(input.charAt(index))) index++; }
        private boolean consume(char expected) {
            if (!end() && input.charAt(index) == expected) { index++; return true; }
            return false;
        }
        private void expect(char expected) { if (!consume(expected)) throw error("expected '" + expected + "'"); }
        private boolean end() { return index >= input.length(); }
        private IllegalArgumentException error(String message) {
            return new IllegalArgumentException("invalid JSON at " + index + ": " + message);
        }
    }
}
