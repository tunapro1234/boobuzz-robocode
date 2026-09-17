package boobuzz.core.contract;

import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.RobotConstants;

import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertThrows;

public class ActionValidatorTest {

    private static final Mechanism MECHANISM = Mechanism.DEFAULT;

    @Test
    public void emptyFrameIsSafeAndValid() {
        ActionValidator.validate(RobotAction.zero(), MECHANISM);
    }

    @Test
    public void unknownNonFiniteAndOutOfRangeValuesAreRejected() {
        assertThrows(ActionValidator.ValidationException.class, () -> validateMotors(
                Map.of("unknown", 0.2)));
        assertThrows(ActionValidator.ValidationException.class, () -> validateMotors(
                Map.of("fl", Double.NaN)));
        assertThrows(ActionValidator.ValidationException.class, () -> validateMotors(
                Map.of("fl", 1.1)));
        assertThrows(ActionValidator.ValidationException.class, () ->
                ActionValidator.validate(new RobotAction(Map.of(), Map.of("hood_left", 0.5)),
                        MECHANISM));
    }

    @Test
    public void pairedDevicesRejectHalfFramesAndMismatchedValues() {
        assertThrows(ActionValidator.ValidationException.class, () -> validateMotors(
                Map.of(RobotConstants.SHOOTER_RIGHT_MOTOR_NAME, 0.5)));
        assertThrows(ActionValidator.ValidationException.class, () -> validateMotors(
                Map.of(RobotConstants.TURRET_PRIMARY_SERVO_NAME, 0.5,
                        RobotConstants.TURRET_SECONDARY_SERVO_NAME, 0.4)));
        assertThrows(ActionValidator.ValidationException.class, () ->
                ActionValidator.validate(new RobotAction(Map.of(), Map.of(
                        RobotConstants.HOOD_LEFT_SERVO_NAME, 0.4,
                        RobotConstants.HOOD_RIGHT_SERVO_NAME, 0.5)), MECHANISM));
        assertThrows(ActionValidator.ValidationException.class, () ->
                ActionValidator.validate(new RobotAction(Map.of(), Map.of(
                        RobotConstants.HOOD_LEFT_SERVO_NAME, 0.2,
                        RobotConstants.HOOD_RIGHT_SERVO_NAME, 0.8)), MECHANISM));
    }

    @Test
    public void completePairsAreAccepted() {
        ActionValidator.validate(new RobotAction(
                Map.of(
                        RobotConstants.SHOOTER_RIGHT_MOTOR_NAME, 0.5,
                        RobotConstants.SHOOTER_LEFT_MOTOR_NAME, 0.5,
                        RobotConstants.TURRET_PRIMARY_SERVO_NAME, -0.25,
                        RobotConstants.TURRET_SECONDARY_SERVO_NAME, -0.25),
                Map.of(
                        RobotConstants.HOOD_LEFT_SERVO_NAME, 0.4266666666666667,
                        RobotConstants.HOOD_RIGHT_SERVO_NAME, 0.5733333333333333)), MECHANISM);
    }

    private static void validateMotors(Map<String, Double> motors) {
        ActionValidator.validate(new RobotAction(motors, Map.of()), MECHANISM);
    }
}
