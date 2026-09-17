package boobuzz.core.contract;

import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.RobotConstants;

import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/** Validates a complete actuator frame before a HAL performs any device write. */
public final class ActionValidator {

    private static final double HOOD_RIGHT_MAX = 215.0 / 300.0;
    private static final double PAIR_EPSILON = 1e-9;

    private ActionValidator() {}

    public static void validate(RobotAction action, Mechanism mechanism) {
        Objects.requireNonNull(action, "action");
        Objects.requireNonNull(mechanism, "mechanism");
        validateMap(action.motors(), new HashSet<>(mechanism.motorNames()),
                "motor", -1.0, 1.0);
        validateMap(action.servos(), new HashSet<>(mechanism.servoNames()),
                "servo", 0.0, 1.0);
        validateEvents(action.events());

        validatePair(action.motors(), RobotConstants.SHOOTER_RIGHT_MOTOR_NAME,
                RobotConstants.SHOOTER_LEFT_MOTOR_NAME, 1.0, false, "shooter");
        validatePair(action.motors(), RobotConstants.TURRET_PRIMARY_SERVO_NAME,
                RobotConstants.TURRET_SECONDARY_SERVO_NAME, 1.0, true, "turret");
        validateHood(action.servos());
    }

    private static void validateMap(Map<String, Double> values, Set<String> declared,
                                    String kind, double minimum, double maximum) {
        if (values == null) {
            throw invalid(kind + " map is null");
        }
        for (Map.Entry<String, Double> entry : values.entrySet()) {
            String name = entry.getKey();
            Double value = entry.getValue();
            if (name == null || !declared.contains(name)) {
                throw invalid("undeclared " + kind + " device: " + name);
            }
            if (value == null || !Double.isFinite(value)
                    || value < minimum || value > maximum) {
                throw invalid(kind + " " + name + " is outside ["
                        + minimum + "," + maximum + "]: " + value);
            }
        }
    }

    private static void validatePair(Map<String, Double> values, String first, String second,
                                     double followerScale, boolean equal, String label) {
        boolean hasFirst = values.containsKey(first);
        boolean hasSecond = values.containsKey(second);
        if (hasFirst != hasSecond) {
            throw invalid(label + " pair must be commanded together");
        }
        if (!hasFirst) {
            return;
        }
        double primary = values.get(first);
        double follower = values.get(second);
        double expected = primary * followerScale;
        if (equal ? Math.abs(primary - follower) > PAIR_EPSILON
                : Math.abs(follower - expected) > PAIR_EPSILON) {
            throw invalid(label + " pair values do not match");
        }
    }

    private static void validateHood(Map<String, Double> values) {
        String left = RobotConstants.HOOD_LEFT_SERVO_NAME;
        String right = RobotConstants.HOOD_RIGHT_SERVO_NAME;
        boolean hasLeft = values.containsKey(left);
        boolean hasRight = values.containsKey(right);
        if (hasLeft != hasRight) {
            throw invalid("hood pair must be commanded together");
        }
        if (!hasLeft) {
            return;
        }
        double leftValue = values.get(left);
        double rightValue = values.get(right);
        if (Math.abs(leftValue + rightValue - 1.0) > PAIR_EPSILON
                || rightValue > HOOD_RIGHT_MAX) {
            throw invalid("hood pair is not a complementary calibrated position");
        }
    }

    private static void validateEvents(java.util.List<Event> events) {
        if (events == null) {
            throw invalid("events list is null");
        }
        for (Event event : events) {
            if (event == null || event.name() == null || event.name().isEmpty()
                    || event.tMs() < 0L) {
                throw invalid("event has invalid name or timestamp");
            }
            if (event.data() == null) {
                throw invalid("event data is null");
            }
            for (Map.Entry<String, Double> value : event.data().entrySet()) {
                if (value.getKey() == null || value.getValue() == null
                        || !Double.isFinite(value.getValue())) {
                    throw invalid("event data contains a non-finite value");
                }
            }
        }
    }

    private static ValidationException invalid(String message) {
        return new ValidationException(message);
    }

    public static final class ValidationException extends IllegalArgumentException {
        public ValidationException(String message) {
            super(message);
        }
    }
}
