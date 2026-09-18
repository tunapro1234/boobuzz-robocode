package boobuzz.core.subsystem.intake;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.hal.RobotConstants;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class PowerIntakeTest {

    @Test
    public void emitsSignedLogicalPowerEveryTickWithoutEvents() {
        PowerIntake intake = new PowerIntake();

        intake.run(1.0);
        assertPower(intake, 1.0);

        intake.run(0.8);
        assertPower(intake, 0.8);

        intake.run(-0.4);
        assertPower(intake, -0.4);

        // A second update at the same power is still a complete actuator frame.
        assertPower(intake, -0.4);
        assertFalse(intake.hasBall());
    }

    @Test
    public void stopEmitsZeroAndClampsUnsafeInputs() {
        PowerIntake intake = new PowerIntake();

        intake.run(2.0);
        assertPower(intake, 1.0);
        intake.run(-2.0);
        assertPower(intake, -1.0);
        intake.run(Double.NaN);
        assertPower(intake, 0.0);
        intake.run(Double.POSITIVE_INFINITY);
        assertPower(intake, 0.0);

        intake.run(0.8);
        intake.stop();
        assertPower(intake, 0.0);
    }

    @Test
    public void keepsHardwareInversionOutOfLogicalCommand() {
        PowerIntake intake = new PowerIntake();
        intake.run(0.8);

        RobotAction.Builder output = new RobotAction.Builder();
        intake.update(output);
        RobotAction action = output.build();

        assertEquals("intake", RobotConstants.INTAKE_MOTOR_NAME);
        assertEquals("REVERSE", RobotConstants.INTAKE.direction());
        assertEquals(1, action.motors().size());
        assertEquals(0.8, action.motor(RobotConstants.INTAKE_MOTOR_NAME), 1e-9);
        assertTrue(action.events().isEmpty());
    }

    private static void assertPower(PowerIntake intake, double expected) {
        RobotAction.Builder output = new RobotAction.Builder();
        intake.update(output);
        RobotAction action = output.build();
        assertEquals(1, action.motors().size());
        assertEquals(expected, action.motor(RobotConstants.INTAKE_MOTOR_NAME), 1e-9);
        assertTrue(action.events().isEmpty());
    }
}
