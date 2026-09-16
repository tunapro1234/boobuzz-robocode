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

    private static RobotState state(long tMs) {
        return new RobotState(tMs, Map.of(), Map.of(), 0.0,
                new Pose(0.0, 0.0, 0.0), 12.6);
    }
}
