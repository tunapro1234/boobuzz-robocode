package boobuzz.core.hal;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * DOWNWARD command. Motor/servo level - NOT subsystem level.
 *
 * <p>The seam is here because the motor interface encodes physics (stable), while
 * the subsystem interface encodes design assumptions (changing). See the architecture documentation, §3.
 *
 * <p>Keys are names from {@code mechanism.yaml}. Adding a motor does not change :core.
 * Values are power from -1..1. A missing key means 0.
 */
public record RobotAction(Map<String, Double> motors, Map<String, Double> servos) {

    public RobotAction {
        motors = Map.copyOf(motors);
        servos = Map.copyOf(servos);
    }

    public static RobotAction zero() {
        return new RobotAction(Collections.emptyMap(), Collections.emptyMap());
    }

    public static RobotAction ofMotors(Map<String, Double> motors) {
        return new RobotAction(motors, Collections.emptyMap());
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public double motor(String name) {
        return motors.getOrDefault(name, 0.0);
    }

    public double servo(String name) {
        return servos.getOrDefault(name, 0.0);
    }

    /** Small helper for manual construction; preserves order (telemetry readability). */
    public static final class Builder {
        private final Map<String, Double> motors = new LinkedHashMap<>();
        private final Map<String, Double> servos = new LinkedHashMap<>();

        public Builder motor(String name, double power) {
            motors.put(name, power);
            return this;
        }

        public Builder servo(String name, double position) {
            servos.put(name, position);
            return this;
        }

        public RobotAction build() {
            return new RobotAction(motors, servos);
        }
    }
}
