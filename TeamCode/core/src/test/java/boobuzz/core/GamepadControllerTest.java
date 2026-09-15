package boobuzz.core;

import boobuzz.core.control.Drive;
import boobuzz.core.control.Feedback;
import boobuzz.core.control.GamepadController;
import boobuzz.core.control.Intent;
import boobuzz.core.control.WorldSnapshot;
import boobuzz.core.hal.GamepadSource;
import boobuzz.core.hal.GamepadState;

import com.pedropathing.math.Pose;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Field-oriented surus: donusum L3'te biter, Drive tipi robot cerceveli kalir. */
public class GamepadControllerTest {

    private static final double EPS = 1e-9;

    /** Elde tutulan gamepad. */
    private static final class Pad implements GamepadSource {
        GamepadState state = GamepadState.neutral();

        @Override public GamepadState get() { return state; }
    }

    /** ly yukari itildiginde negatif; "ileri stick" budur. */
    private static GamepadState forwardStick() {
        return new GamepadState(0, -1, 0, 0,
                false, false, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
    }

    private static Feedback at(double heading) {
        return Feedback.of(new WorldSnapshot(0, new Pose(72, 72, heading), heading, 12.6));
    }

    private static Drive.Manual driveOf(Intent intent) {
        assertTrue("Drive.Manual bekleniyor", intent.drive() instanceof Drive.Manual);
        return (Drive.Manual) intent.drive();
    }

    @Test
    public void varsayilanFieldOrientedAcik() {
        assertTrue(new GamepadController(new Pad()).fieldOriented());
    }

    @Test
    public void heading0daIleriStickDegismez() {
        Pad pad = new Pad();
        pad.state = forwardStick();
        Drive.Manual d = driveOf(new GamepadController(pad).decide(at(0)));
        assertEquals(1.0, d.vx(), EPS);
        assertEquals(0.0, d.vy(), EPS);
    }

    @Test
    public void headingYarimPideIleriStickVyYonuneDoner() {
        Pad pad = new Pad();
        pad.state = forwardStick();
        Drive.Manual d = driveOf(new GamepadController(pad).decide(at(Math.PI / 2)));
        // Robot saha +y'ye bakiyor; saha +x istegi robotun SAGI, yani vy negatif.
        assertEquals(0.0, d.vx(), EPS);
        assertEquals(-1.0, d.vy(), EPS);
    }

    @Test
    public void robotOrientedModdaHeadingOnemsiz() {
        Pad pad = new Pad();
        GamepadController c = new GamepadController(pad);
        pad.state = new GamepadState(0, -1, 0, 0,
                false, true, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
        Drive.Manual d = driveOf(c.decide(at(Math.PI / 2)));
        assertFalse(c.fieldOriented());
        assertEquals(1.0, d.vx(), EPS);
        assertEquals(0.0, d.vy(), EPS);
    }

    @Test
    public void toggleKenardaTetiklenirBasiliTutmaSaymaz() {
        Pad pad = new Pad();
        GamepadController c = new GamepadController(pad);
        GamepadState bHeld = new GamepadState(0, 0, 0, 0,
                false, true, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
        pad.state = bHeld;
        c.decide(at(0));
        assertFalse(c.fieldOriented());
        c.decide(at(0));
        c.decide(at(0));
        assertFalse("basili tutmak modu geri cevirmemeli", c.fieldOriented());
        pad.state = GamepadState.neutral();
        c.decide(at(0));
        pad.state = bHeld;
        c.decide(at(0));
        assertTrue(c.fieldOriented());
    }

    @Test
    public void ySifirlamasiOAnkiHeadingiIleriYapar() {
        Pad pad = new Pad();
        GamepadController c = new GamepadController(pad);
        pad.state = new GamepadState(0, -1, 0, 0,
                false, false, false, true, false, false, 0, 0, GamepadState.Dpad.NONE);
        Drive.Manual d = driveOf(c.decide(at(Math.PI / 2)));
        assertEquals(Math.PI / 2, c.headingOffset(), EPS);
        // Sifirlamadan sonra ayni tick'te bile robot cercevesi = saha cercevesi.
        assertEquals(1.0, d.vx(), EPS);
        assertEquals(0.0, d.vy(), EPS);
    }

    @Test
    public void donusHerIkiModdaRobotCerceveli() {
        Pad pad = new Pad();
        GamepadState turn = new GamepadState(0, 0, -1, 0,
                false, false, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
        pad.state = turn;
        assertEquals(1.0, driveOf(new GamepadController(pad).decide(at(Math.PI / 3))).omega(), EPS);
    }

    @Test
    public void pozYoksaYawKullanilir() {
        Pad pad = new Pad();
        pad.state = forwardStick();
        Feedback noPose = Feedback.of(new WorldSnapshot(0, null, Math.PI / 2, 12.6));
        Drive.Manual d = driveOf(new GamepadController(pad).decide(noPose));
        assertEquals(0.0, d.vx(), EPS);
        assertEquals(-1.0, d.vy(), EPS);
    }
}
