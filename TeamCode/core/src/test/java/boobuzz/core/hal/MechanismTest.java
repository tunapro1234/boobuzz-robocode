package boobuzz.core.hal;

import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public class MechanismTest {

    private static Mechanism testMechanism() {
        return Mechanism.DEFAULT;
    }

    @Test
    public void motorNamesPreserveOrder() {
        assertEquals(List.of("fl", "fr", "bl", "br"), testMechanism().motorNames());
        assertEquals(4, RobotConstants.MOTORS.length);
    }

    @Test
    public void robotDimensionsAndPhysicsComeFromConstants() {
        assertEquals(18.0, RobotConstants.ROBOT_WIDTH, 1e-9);
        assertEquals(18.0, RobotConstants.ROBOT_LENGTH, 1e-9);
        assertEquals("deg", RobotConstants.ANGLE_UNIT);
        assertEquals("mecanum", RobotConstants.DRIVETRAIN_TYPE);
        assertEquals(4.0, RobotConstants.WHEEL_DIAMETER, 1e-9);
        assertEquals(12.0, RobotConstants.BATTERY_V, 1e-9);
        assertEquals(0.1, RobotConstants.MOTOR_TAU_S, 1e-9);
        assertEquals(0.7346, RobotConstants.STRAFE_EFF, 1e-9);
        assertEquals(36.17, RobotConstants.ZERO_POWER_DECEL_FORWARD_IN_S2, 1e-9);
        assertEquals(85.98, RobotConstants.ZERO_POWER_DECEL_LATERAL_IN_S2, 1e-9);
        assertEquals(1.0, RobotConstants.EFFICIENCY_FL, 1e-9);
        assertEquals(1.0, RobotConstants.EFFICIENCY_FR, 1e-9);
        assertEquals(1.0, RobotConstants.EFFICIENCY_BL, 1e-9);
        assertEquals(1.0, RobotConstants.EFFICIENCY_BR, 1e-9);
    }

    @Test
    public void motorConstantsPreserveAllFields() {
        RobotConstants.Motor fl = RobotConstants.FL;
        assertEquals("fl", fl.name());
        assertEquals("wheel", fl.drives());
        assertEquals(6.5, fl.xForward(), 1e-9);
        assertEquals(5.5, fl.yLeft(), 1e-9);
        assertEquals(45.0, fl.rollerDeg(), 1e-9);
        assertEquals(537.7, fl.ticksPerRev(), 1e-9);
        assertEquals(351.55735379568756, fl.freeRpm(), 1e-12);

        RobotConstants.Motor br = RobotConstants.BR;
        assertEquals(-6.5, br.xForward(), 1e-9);
        assertEquals(-5.5, br.yLeft(), 1e-9);
        assertEquals(45.0, br.rollerDeg(), 1e-9);
    }

    @Test
    public void pinpointSettingsComeFromConstants() {
        RobotConstants.Pinpoint p = RobotConstants.PINPOINT;
        assertEquals(161.0, p.xPodOffsetMm(), 1e-9);
        assertEquals(0.0, p.yPodOffsetMm(), 1e-9);
        assertEquals("FORWARD", p.xPodDirection());
        assertEquals("REVERSED", p.yPodDirection());
        assertEquals("goBILDA_4_BAR_POD", p.podType());

        Mechanism.Pinpoint mechanismPinpoint = testMechanism().pinpoint();
        assertEquals(p.xPodOffsetMm(), mechanismPinpoint.xPodOffsetMm(), 1e-9);
        assertEquals(p.yPodOffsetMm(), mechanismPinpoint.yPodOffsetMm(), 1e-9);
        assertEquals(p.xPodDirection(), mechanismPinpoint.xPodDirection());
        assertEquals(p.yPodDirection(), mechanismPinpoint.yPodDirection());
        assertEquals(p.podType(), mechanismPinpoint.podType());
    }

    @Test
    public void physicsDataIsRead() {
        Mechanism.Physics physics = testMechanism().physics();
        assertEquals(RobotConstants.STRAFE_EFF, physics.strafeEfficiency(), 1e-9);
        assertEquals(RobotConstants.EFFICIENCY_FL, physics.efficiency().get("fl"), 1e-9);
    }

    @Test
    public void defaultMechanismIsBuiltOnce() {
        assertSame(Mechanism.DEFAULT, RobotConstants.mechanism());
        assertEquals(4.0, testMechanism().drivetrain().wheelDiameter(), 1e-9);
        assertEquals(4, testMechanism().wheelMotorNames().size());
    }

    @Test
    public void servosAreEmptyForNow() {
        assertEquals(0, RobotConstants.SERVOS.length);
        assertTrue(testMechanism().servoNames().isEmpty());
    }

    @Test
    public void positionIsInterpretedAsForwardAndLeft() {
        Mechanism m = testMechanism();
        assertEquals(6.5, m.motor("fl").forward(), 1e-9);
        assertEquals(5.5, m.motor("fl").left(), 1e-9);
        assertEquals(-6.5, m.motor("br").forward(), 1e-9);
        assertEquals(-5.5, m.motor("br").left(), 1e-9);
    }

    @Test
    public void nameListMismatchFails() {
        Mechanism m = testMechanism();
        m.requireNames(List.of("br", "bl", "fr", "fl"), List.of()); // order does not matter
        Mechanism.MechanismException e = assertThrows(Mechanism.MechanismException.class,
                () -> m.requireNames(List.of("fl", "fr", "bl"), List.of()));
        assertTrue(e.getMessage().contains("RobotConstants"));
    }
}
