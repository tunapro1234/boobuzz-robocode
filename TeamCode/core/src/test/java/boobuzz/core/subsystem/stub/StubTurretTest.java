package boobuzz.core.subsystem.stub;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class StubTurretTest {

    @Test
    public void aimSettlesAndEmitsLockThenScan() {
        StubTurret turret = new StubTurret();
        turret.observe(state(0));
        turret.aimAt(10.0, 10.0);
        assertFalse(turret.onTarget());

        turret.observe(state(Math.round(RobotConstants.STUB_TURRET_SETTLE_S * 1000.0) - 1));
        assertFalse(turret.onTarget());
        turret.observe(state(Math.round(RobotConstants.STUB_TURRET_SETTLE_S * 1000.0)));
        assertTrue(turret.onTarget());

        RobotAction.Builder lockedOut = new RobotAction.Builder();
        turret.update(lockedOut);
        assertEquals("turret.locked", lockedOut.build().events().get(0).name());

        turret.scan();
        RobotAction.Builder scanOut = new RobotAction.Builder();
        turret.update(scanOut);
        assertEquals("turret.scan", scanOut.build().events().get(0).name());
        assertFalse(turret.onTarget());
    }

    @Test
    public void holdCancelsPendingAimSettleAndTransitionEvents() {
        StubTurret turret = new StubTurret();
        long settleMs = Math.round(RobotConstants.STUB_TURRET_SETTLE_S * 1000.0);
        turret.observe(state(0));
        turret.aimAt(10.0, 10.0);
        turret.observe(state(settleMs));
        turret.hold();
        turret.observe(state(settleMs + 100));

        assertFalse("a cancelled aim must not become locked later", turret.onTarget());
        RobotAction.Builder out = new RobotAction.Builder();
        turret.update(out);
        assertTrue("cancelled transitions must not emit stale events",
                out.build().events().isEmpty());
    }

    @Test
    public void holdClearsPendingScanEvent() {
        StubTurret turret = new StubTurret();
        turret.observe(state(0));
        turret.scan();
        turret.hold();

        RobotAction.Builder out = new RobotAction.Builder();
        turret.update(out);
        assertTrue("hold must clear a queued scan event", out.build().events().isEmpty());
    }

    private static RobotState state(long tMs) {
        return new RobotState(tMs, Map.of(), Map.of(), 0.0,
                new Pose(0.0, 0.0, 0.0), 12.6);
    }
}
