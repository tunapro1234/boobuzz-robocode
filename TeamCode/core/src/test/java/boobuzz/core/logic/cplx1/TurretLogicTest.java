package boobuzz.core.logic.cplx1;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.ITurret;

import com.pedropathing.math.Pose;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class TurretLogicTest {

    @Test
    public void aimsAtAllianceGoalAndFreezesForShot() {
        RecordingTurret turret = new RecordingTurret();
        TurretLogic logic = new TurretLogic(turret);
        logic.update(new Pose(24.0, 48.0, 0.0));
        assertEquals(RobotConstants.GOAL_X, turret.x, 1e-9);
        assertEquals(RobotConstants.GOAL_Y, turret.y, 1e-9);
        assertTrue(logic.locked());

        logic.holdForShot(true);
        logic.update(new Pose(80.0, 80.0, 0.0));
        assertEquals(RobotConstants.GOAL_X, turret.x, 1e-9);
        assertEquals(RobotConstants.GOAL_Y, turret.y, 1e-9);
        assertTrue(turret.held);
    }

    private static final class RecordingTurret implements ITurret {
        double x;
        double y;
        boolean held;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void aimAt(double fieldX, double fieldY) { x = fieldX; y = fieldY; }
        @Override public void scan() {}
        @Override public void hold() { held = true; }
        @Override public boolean onTarget() { return true; }
        @Override public double angleRad() { return 0.0; }
    }
}
