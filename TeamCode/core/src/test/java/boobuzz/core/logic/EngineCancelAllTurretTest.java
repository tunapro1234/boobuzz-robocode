package boobuzz.core.logic;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.logic.cplx1.CplxEngine1;
import boobuzz.core.logic.direct.DirectEngine;
import boobuzz.core.subsystem.IDrive;
import boobuzz.core.subsystem.IShooter;
import boobuzz.core.subsystem.ITurret;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.stub.StubIntake;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class EngineCancelAllTurretTest {

    @Test
    public void cplxCancelAllDisablesTurretUntilFreshRequest() {
        RecordingTurret turret = new RecordingTurret();
        CplxEngine1 engine = new CplxEngine1(
                new Subsystems(new RecordingDrive(), new RecordingShooter(),
                        new StubIntake(), turret));
        engine.sense(state());
        assertTrue(turret.aimCalls > 0);

        engine.act(RequestBatch.cancelAll());
        assertTrue(turret.disableCalls > 0);

        int aimsAfterCancel = turret.aimCalls;
        engine.sense(state());
        engine.act(RequestBatch.idle());
        assertEquals("idle sense must not re-arm a cancelled target", aimsAfterCancel, turret.aimCalls);

        engine.act(RequestBatch.of(boobuzz.core.contract.Request.turretAim(9, 48.0, 96.0)));
        engine.sense(state());
        assertTrue(turret.aimCalls > aimsAfterCancel);
    }

    @Test
    public void directCancelAllAlsoDisablesTurret() {
        RecordingTurret turret = new RecordingTurret();
        DirectEngine engine = new DirectEngine(
                new Subsystems(new RecordingDrive(), new RecordingShooter(),
                        new StubIntake(), turret));

        engine.act(RequestBatch.cancelAll());

        assertTrue(turret.disableCalls > 0);
    }

    private static RobotState state() {
        return new RobotState(0L, Map.of(), Map.of(), 0.0,
                new Pose(24.0, 48.0, 0.0), 12.0);
    }

    private static final class RecordingDrive implements IDrive {
        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void manual(double vx, double vy, double omega) {}
        @Override public void follow(PathRequest request) {}
        @Override public void stop() {}
        @Override public boolean pathDone() { return true; }
        @Override public Pose pose() { return new Pose(24.0, 48.0, 0.0); }
    }

    private static final class RecordingShooter implements IShooter {
        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void spinUp(double rpm) {}
        @Override public void spinDown() {}
        @Override public boolean isReady() { return false; }
        @Override public void feed() {}
        @Override public boolean isFeeding() { return false; }
        @Override public void setHoodAngleDeg(double angleDeg) {}
        @Override public boolean hoodSettled() { return false; }
        @Override public void runOpenLoop(double power) {}
        @Override public void setFeederPower(double power) {}
        @Override public boolean isStopped() { return true; }
    }

    private static final class RecordingTurret implements ITurret {
        int aimCalls;
        int holdCalls;
        int disableCalls;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void setRobotPose(com.pedropathing.math.Pose pose) {}
        @Override public void aimAt(double fieldX, double fieldY) { aimCalls++; }
        @Override public void scan() {}
        @Override public void hold() { holdCalls++; }
        @Override public boolean onTarget() { return true; }
        @Override public double angleRad() { return 0.0; }
        @Override public AimResult aimRelative(double angleRad) { return AimResult.ACCEPTED; }
        @Override public AimResult aimStatus() { return AimResult.ACCEPTED; }
        @Override public void disable() { disableCalls++; }
    }
}
