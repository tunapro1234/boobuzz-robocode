package boobuzz.core.logic.cplx1;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.ITurret;

import com.pedropathing.math.Pose;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
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

    @Test
    public void explicitFieldTargetSurvivesUpdates() {
        RecordingTurret turret = new RecordingTurret();
        TurretLogic logic = new TurretLogic(turret);
        logic.setFieldTarget(30.0, 100.0);
        logic.update(new Pose(24.0, 48.0, 0.0));
        logic.update(new Pose(26.0, 50.0, 0.0));
        assertEquals(TurretLogic.TargetMode.FIELD_POINT, logic.targetMode());
        assertEquals(30.0, turret.x, 1e-9);
        assertEquals(100.0, turret.y, 1e-9);
        assertEquals(Math.hypot(4.0, 50.0), logic.distanceFrom(new Pose(26.0, 50.0, 0.0)), 1e-9);

        logic.useAllianceGoal();
        logic.update(new Pose(24.0, 48.0, 0.0));
        assertEquals(RobotConstants.GOAL_X, turret.x, 1e-9);
    }

    @Test
    public void relativeTargetUsesAimRelativeAndHasNoDistance() {
        RecordingTurret turret = new RecordingTurret();
        TurretLogic logic = new TurretLogic(turret);
        logic.setRelativeTarget(Math.toRadians(30.0));
        logic.update(null);
        assertEquals(Math.toRadians(30.0), turret.relative, 1e-12);
        assertEquals(0, turret.aimAtCalls);
        assertTrue(logic.locked());
        assertEquals(0.0, logic.distanceFrom(new Pose(24.0, 48.0, 0.0)), 0.0);
    }

    @Test
    public void rejectedAimIsNeverLocked() {
        RecordingTurret turret = new RecordingTurret();
        TurretLogic logic = new TurretLogic(turret);
        logic.update(new Pose(24.0, 48.0, 0.0));
        turret.status = ITurret.AimResult.OUT_OF_RANGE;
        assertFalse(logic.locked());
        turret.status = ITurret.AimResult.NOT_INITIALIZED;
        assertFalse(logic.locked());
    }

    @Test
    public void unknownPoseScansAndIsNotLocked() {
        RecordingTurret turret = new RecordingTurret();
        TurretLogic logic = new TurretLogic(turret);
        logic.update(null);
        assertTrue(turret.scanned);
        assertFalse(logic.locked());
        assertEquals(0.0, logic.distanceFrom(new Pose(0.0, 0.0, 0.0)), 0.0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void nonFiniteFieldTargetIsRejected() {
        new TurretLogic(new RecordingTurret()).setFieldTarget(Double.NaN, 0.0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void nonFiniteRelativeTargetIsRejected() {
        new TurretLogic(new RecordingTurret()).setRelativeTarget(Double.POSITIVE_INFINITY);
    }

    private static final class RecordingTurret implements ITurret {
        double x;
        double y;
        double relative = Double.NaN;
        int aimAtCalls;
        boolean held;
        boolean scanned;
        AimResult status = AimResult.ACCEPTED;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void aimAt(double fieldX, double fieldY) { x = fieldX; y = fieldY; aimAtCalls++; }
        @Override public void scan() { scanned = true; }
        @Override public void hold() { held = true; }
        @Override public boolean onTarget() { return true; }
        @Override public double angleRad() { return 0.0; }
        @Override public AimResult aimRelative(double angleRad) { relative = angleRad; return status; }
        @Override public AimResult aimStatus() { return status; }
        @Override public void disable() {}
    }
}
