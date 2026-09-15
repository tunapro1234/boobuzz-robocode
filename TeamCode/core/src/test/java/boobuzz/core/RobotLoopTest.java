package boobuzz.core;

import boobuzz.core.controller.Controller;
import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Feedback;
import boobuzz.core.controller.GamepadController;
import boobuzz.core.contract.Intent;
import boobuzz.core.logic.engine.C1DriveEngine;
import boobuzz.core.hal.GamepadState;
import boobuzz.core.hal.Hal;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;
import boobuzz.core.hal.Mechanism;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class RobotLoopTest {

    /** Kayit tutan sahte HAL. Soket yok, Python yok. */
    private static final class FakeHal implements Hal {
        long t = 0;
        final int dtMs = 20;
        GamepadState pad = GamepadState.neutral();
        final List<RobotAction> written = new ArrayList<>();

        @Override public long now() { return t; }

        @Override public RobotState read() {
            return new RobotState(t, Map.of(), Map.of(), 0, new Pose(0, 0, 0), 12.6);
        }

        @Override public void write(RobotAction action) {
            written.add(action);
            t += dtMs;
        }

        @Override public GamepadState get() { return pad; }
    }

    private static Mechanism mechanism() {
        try (InputStream in = RobotLoopTest.class.getResourceAsStream("/mechanism-test.yaml")) {
            return Mechanism.load(in, "mechanism-test.yaml");
        } catch (Exception e) {
            throw new AssertionError(e);
        }
    }

    @Test
    public void tickZamaniHaldenAlir() {
        FakeHal hal = new FakeHal();
        RobotLoop loop = new RobotLoop(hal, new C1DriveEngine(mechanism()),
                fb -> Intent.of(new Drive.Manual(1, 0, 0)));

        for (int i = 0; i < 10; i++) {
            loop.tick();
        }
        assertEquals(10, loop.ticks());
        assertEquals(200, hal.now());
        assertEquals(10, hal.written.size());
        assertEquals(1.0, hal.written.get(9).motor("fl"), 1e-9);
    }

    @Test
    public void gamepadControllerStickleriDogruCevirir() {
        FakeHal hal = new FakeHal();
        // Stick yukari itilince ly negatif olur; ileri surus beklenir.
        hal.pad = new GamepadState(0, -1.0, 0, 0,
                false, false, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);

        Controller controller = new GamepadController(hal);
        Intent intent = controller.decide(null);
        assertTrue(intent.drive() instanceof Drive.Manual);
        Drive.Manual m = (Drive.Manual) intent.drive();
        assertEquals(1.0, m.vx(), 1e-9);
        assertEquals(0.0, m.vy(), 1e-9);
        assertEquals(0.0, m.omega(), 1e-9);
    }

    @Test
    public void oluBolgeMerkezdeSifirVerir() {
        FakeHal hal = new FakeHal();
        hal.pad = new GamepadState(0.02, -0.03, 0.01, 0,
                false, false, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
        Drive.Manual m = (Drive.Manual) new GamepadController(hal).decide(null).drive();
        assertEquals(0.0, m.vx(), 1e-9);
        assertEquals(0.0, m.vy(), 1e-9);
        assertEquals(0.0, m.omega(), 1e-9);
    }

    @Test
    public void ucUctanUcaGamepadMotoraVarir() {
        FakeHal hal = new FakeHal();
        hal.pad = new GamepadState(0, -1.0, 0, 0,
                false, false, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
        RobotLoop loop = new RobotLoop(hal, new C1DriveEngine(mechanism()),
                new GamepadController(hal));
        loop.tick();

        RobotAction a = hal.written.get(0);
        assertEquals(1.0, a.motor("fl"), 1e-9);
        assertEquals(1.0, a.motor("fr"), 1e-9);
        assertEquals(1.0, a.motor("bl"), 1e-9);
        assertEquals(1.0, a.motor("br"), 1e-9);
    }
}
