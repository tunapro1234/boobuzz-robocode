package boobuzz.core.hal;

import boobuzz.core.hal.Mechanism;

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
        return Mechanism.load(in, "<test>");
    }

    private static Mechanism testMechanism() {
        try (InputStream in = MechanismTest.class.getResourceAsStream("/mechanism-test.yaml")) {
            return Mechanism.load(in, "mechanism-test.yaml");
        } catch (Exception e) {
            throw new AssertionError(e);
        }
    }

    @Test
    public void motorAdlariSirayiKorur() {
        assertEquals(List.of("fl", "fr", "bl", "br"), testMechanism().motorNames());
    }

    @Test
    public void aciDereceOkunurRadyanVerilir() {
        Mechanism m = testMechanism();
        assertEquals(Math.toRadians(45), m.motor("fl").rollerRad(), 1e-12);
        assertEquals(Math.toRadians(-45), m.motor("fr").rollerRad(), 1e-12);
    }

    @Test
    public void drivetrainOlculeriOkunur() {
        Mechanism.Drivetrain d = testMechanism().drivetrain();
        assertEquals("mecanum", d.type());
        assertEquals(13.0, d.trackWidth(), 1e-9);
        assertEquals(11.0, d.wheelBase(), 1e-9);
        assertEquals(4.0, d.wheelDiameter(), 1e-9);
    }

    @Test
    public void pinpointAyarlariTekKaynaktanOkunur() {
        Mechanism.Pinpoint p = testMechanism().pinpoint();
        assertEquals(161.0, p.xPodOffsetMm(), 1e-9);
        assertEquals(0.0, p.yPodOffsetMm(), 1e-9);
        assertEquals("FORWARD", p.xPodDirection());
        assertEquals("REVERSED", p.yPodDirection());
        assertEquals("goBILDA_4_BAR_POD", p.podType());
    }

    @Test
    public void projedekiGercekDosyaOkunabiliyor() throws Exception {
        Path p = Path.of("..", "..", "mechanism.yaml").toAbsolutePath().normalize();
        assertTrue("mechanism.yaml bulunamadi: " + p, Files.exists(p));
        Mechanism m = Mechanism.load(p);
        assertEquals(4, m.wheelMotorNames().size());
    }

    @Test
    public void gercekDosyaCoreJarKaynaklarindanOkunabiliyor() {
        assertEquals(4, Mechanism.loadDefault().wheelMotorNames().size());
    }

    @Test
    public void freeRpmZorunlu() {
        Mechanism.MechanismException e = assertThrows(Mechanism.MechanismException.class,
                () -> load("""
                        motors:
                          fl: {drives: wheel, pos: [0, 0]}
                        """));
        assertTrue(e.getMessage().contains("free_rpm"));
    }

    @Test
    public void robotAyakIziOkunur() {
        Mechanism m = testMechanism();
        assertEquals(18.0, m.robot().width(), 1e-9);
        assertEquals(18.0, m.robot().length(), 1e-9);
    }

    @Test
    public void posIleriVeSolOlarakYorumlanir() {
        Mechanism m = testMechanism();
        assertEquals(6.5, m.motor("fl").forward(), 1e-9);
        assertEquals(5.5, m.motor("fl").left(), 1e-9);
        assertEquals(-6.5, m.motor("br").forward(), 1e-9);
        assertEquals(-5.5, m.motor("br").left(), 1e-9);
    }

    @Test
    public void adListesiUyusmazligindaCoker() {
        Mechanism m = testMechanism();
        m.requireNames(List.of("br", "bl", "fr", "fl"), List.of()); // sira onemsiz
        Mechanism.MechanismException e = assertThrows(Mechanism.MechanismException.class,
                () -> m.requireNames(List.of("fl", "fr", "bl"), List.of()));
        assertTrue(e.getMessage().contains("uyusmuyor"));
    }

    @Test
    public void tanimsizEbeveynCerceveHataVerir() {
        Mechanism.MechanismException e = assertThrows(Mechanism.MechanismException.class,
                () -> load("""
                        frames:
                          camera: {parent: turret}
                        motors:
                          fl: {drives: wheel, pos: [0, 0], free_rpm: 312}
                        """));
        assertTrue(e.getMessage().contains("turret"));
    }

    @Test
    public void motorsuzDosyaHataVerir() {
        assertThrows(Mechanism.MechanismException.class,
                () -> load("frames:\n  robot: {parent: field}\n"));
    }

    @Test
    public void tfAgaciVeFovOkunur() {
        Mechanism m = load("""
                units: {length: in, angle: deg}
                frames:
                  robot:  {parent: field}
                  turret: {parent: robot, xyz: [0, 1.5, 8], joint: revolute, axis: z, limits: [-180, 180]}
                  camera: {parent: turret, xyz: [0, 4, 2], rpy: [0, -20, 0], hfov: 63.3, vfov: 49.7}
                motors:
                  fl: {drives: wheel, pos: [0, 0], free_rpm: 312}
                """);
        Mechanism.Frame turret = m.frames().get("turret");
        assertEquals("robot", turret.parent());
        assertEquals("revolute", turret.joint());
        assertEquals(Math.toRadians(-180), turret.limitsRad()[0], 1e-12);
        Mechanism.Frame cam = m.frames().get("camera");
        assertEquals(Math.toRadians(-20), cam.rpyRad()[1], 1e-12);
        assertEquals(Math.toRadians(63.3), cam.hfovRad(), 1e-12);
    }
}
