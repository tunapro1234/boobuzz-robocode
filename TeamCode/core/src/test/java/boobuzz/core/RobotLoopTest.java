package boobuzz.core;

import boobuzz.core.controller.teleop.TeleopController;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.logic.cplx1.CplxEngine1;
import boobuzz.core.contract.GamepadState;
import boobuzz.core.hal.IHal;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.subsystem.stub.StubIntake;
import boobuzz.core.subsystem.stub.StubShooter;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.pedro.PedroDrive;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

public class RobotLoopTest {

    /** Recording fake HAL. No socket or Python. */
    private static final class FakeHal implements IHal {
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

    private static Subsystems subsystems() {
        return new Subsystems(new PedroDrive(Mechanism.DEFAULT),
                new StubShooter(), new StubIntake());
    }

    @Test
    public void tickTimeComesFromHal() {
        FakeHal hal = new FakeHal();
        RobotLoop loop = new RobotLoop(hal, new CplxEngine1(subsystems()),
                fb -> new RequestBatch(RequestStream.manual(1, 0, 0), List.of(), new int[0]));

        for (int i = 0; i < 10; i++) {
            loop.tick();
        }
        assertEquals(10, loop.ticks());
        assertEquals(200, hal.now());
        assertEquals(10, hal.written.size());
        assertEquals(1.0, hal.written.get(9).motor("fl"), 1e-9);
    }

    @Test
    public void deadbandReturnsZeroAtCenter() {
        FakeHal hal = new FakeHal();
        hal.pad = new GamepadState(0.02, -0.03, 0.01, 0,
                false, false, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
        RequestStream m = new TeleopController(hal).decide(null).stream();
        assertEquals(0.0, m.vx(), 1e-9);
        assertEquals(0.0, m.vy(), 1e-9);
        assertEquals(0.0, m.omega(), 1e-9);
        assertFalse(m.manualDrive());
    }

    @Test
    public void gamepadReachesMotorsEndToEnd() {
        FakeHal hal = new FakeHal();
        hal.pad = new GamepadState(0, -1.0, 0, 0,
                false, false, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
        RobotLoop loop = new RobotLoop(hal, new CplxEngine1(subsystems()),
                new TeleopController(hal));
        loop.tick();

        RobotAction a = hal.written.get(0);
        assertEquals(1.0, a.motor("fl"), 1e-9);
        assertEquals(1.0, a.motor("fr"), 1e-9);
        assertEquals(1.0, a.motor("bl"), 1e-9);
        assertEquals(1.0, a.motor("br"), 1e-9);
    }
}
