package boobuzz.core.hal;

import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertEquals;

/** Binding-level assertions for the complete B01 actuator and encoder profile. */
public class HardwareProfileTest {

    @Test
    public void declaresAllOutputsInStableWireOrder() {
        Mechanism mechanism = Mechanism.DEFAULT;
        assertEquals(List.of("fl", "fr", "bl", "br", "intake", "feeder",
                "shooterRight", "shooterLeft", "turret_servo", "turret_servo2"),
                mechanism.motorNames());
        assertEquals(List.of("hood_left", "hood_right"), mechanism.servoNames());
        assertEquals(List.of("leftFront", "rightFront", "leftBack", "rightBack",
                "intake", "feeder", "shooterRight", "shooterLeft"),
                mechanism.encoderNames());
    }

    @Test
    public void sharedShooterLeftPortHasIndependentOutputAndTurretInputRoles() {
        Mechanism mechanism = Mechanism.DEFAULT;
        assertEquals("shooterLeft", RobotConstants.SHOOTER_LEFT_MOTOR_NAME);
        assertEquals("FORWARD", mechanism.dcDevice("shooterLeft").direction());
        assertEquals("FLOAT", mechanism.dcDevice("shooterLeft").zeroPower());
        assertEquals("shooterRight", RobotConstants.SHOOTER_FEEDBACK_ENCODER_NAME);
        assertEquals("shooterLeft", RobotConstants.TURRET_ENCODER_NAME);
        assertEquals(1, mechanism.encoderNames().stream()
                .filter("shooterLeft"::equals).count());
    }

    @Test
    public void actuatorDirectionAndInitialPositionDeclarationsAreExplicit() {
        Mechanism mechanism = Mechanism.DEFAULT;
        assertEquals("REVERSE", mechanism.dcDevice("intake").direction());
        assertEquals("BRAKE", mechanism.dcDevice("intake").zeroPower());
        assertEquals("FORWARD", mechanism.dcDevice("feeder").direction());
        assertEquals("BRAKE", mechanism.dcDevice("feeder").zeroPower());
        assertEquals("REVERSE", mechanism.positionalServo("hood_left").direction());
        assertEquals("FORWARD", mechanism.positionalServo("hood_right").direction());
        assertEquals(1.0, mechanism.positionalServo("hood_left").initialPos(), 1e-9);
        assertEquals(0.0, mechanism.positionalServo("hood_right").initialPos(), 1e-9);
    }
}
