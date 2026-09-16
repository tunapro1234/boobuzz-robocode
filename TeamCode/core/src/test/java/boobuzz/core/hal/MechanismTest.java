package boobuzz.core.hal;

import org.junit.Test;

import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public class MechanismTest {

    private static Mechanism load(String yaml) {
        InputStream in = new ByteArrayInputStream(yaml.getBytes(StandardCharsets.UTF_8));
        return MechanismLoader.load(in, "<test>");
    }

    private static Mechanism testMechanism() {
        try (InputStream in = MechanismTest.class.getResourceAsStream("/mechanism-test.yaml")) {
            return MechanismLoader.load(in, "mechanism-test.yaml");
        } catch (Exception e) {
            throw new AssertionError(e);
        }
    }

    @Test
    public void motorNamesPreserveOrder() {
        assertEquals(List.of("fl", "fr", "bl", "br"), testMechanism().motorNames());
    }

    @Test
    public void drivetrainDimensionsAreRead() {
        Mechanism.Drivetrain d = testMechanism().drivetrain();
        assertEquals(4.0, d.wheelDiameter(), 1e-9);
    }

    @Test
    public void pinpointSettingsComeFromSingleSource() {
        Mechanism.Pinpoint p = testMechanism().pinpoint();
        assertEquals(161.0, p.xPodOffsetMm(), 1e-9);
        assertEquals(0.0, p.yPodOffsetMm(), 1e-9);
        assertEquals("FORWARD", p.xPodDirection());
        assertEquals("REVERSED", p.yPodDirection());
        assertEquals("goBILDA_4_BAR_POD", p.podType());
    }

    @Test
    public void physicsDataIsRead() {
        Mechanism.Physics physics = testMechanism().physics();
        assertEquals(0.7346, physics.strafeEfficiency(), 1e-9);
        assertEquals(1.0, physics.efficiency().get("fl"), 1e-9);
    }

    @Test
    public void projectMechanismFileCanBeRead() throws Exception {
        Path p = Path.of("..", "..", "mechanism.yaml").toAbsolutePath().normalize();
        assertTrue("mechanism.yaml not found: " + p, Files.exists(p));
        Mechanism m = MechanismLoader.load(p);
        assertEquals(4, m.wheelMotorNames().size());
    }

    @Test
    public void projectFileCanBeReadFromCoreJarResources() {
        assertEquals(4, MechanismLoader.loadDefault().wheelMotorNames().size());
    }

    @Test
    public void freeRpmIsRequired() {
        Mechanism.MechanismException e = assertThrows(Mechanism.MechanismException.class,
                () -> load("""
                        motors:
                          fl: {drives: wheel, pos: [0, 0]}
                        """));
        assertTrue(e.getMessage().contains("free_rpm"));
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
        assertTrue(e.getMessage().contains("do not match"));
    }

    @Test
    public void fileWithoutMotorsFails() {
        assertThrows(Mechanism.MechanismException.class,
                () -> load("frames:\n  robot: {parent: field}\n"));
    }

}
