package boobuzz.core.logic.cplx1;

import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.subsystem.IShooter;
import boobuzz.core.subsystem.ITurret;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;

public class ShooterLogicTest {

    @Test
    public void shotWaitsForReadyAndReportsDone() {
        RecordingShooter shooter = new RecordingShooter();
        RecordingTurret turret = new RecordingTurret();
        ShooterLogic logic = new ShooterLogic(shooter, turretLogic(turret));
        // Stationary gate: a full 200 ms window plus the 150 ms hold.
        for (long t = 0; t <= 360; t += 20) {
            logic.observe(t, new Pose(24.0, 48.0, 0.0));
        }
        RequestStatus accepted = logic.requestShot(4, 1);
        assertEquals(RequestStatus.State.ACTIVE, accepted.state());
        assertEquals(RequestStatus.State.REJECTED, logic.requestShot(5, 1).state());

        shooter.ready = true;
        List<RequestStatus> statuses = new ArrayList<>();
        logic.update(statuses);
        assertEquals(RequestStatus.State.ACTIVE, statuses.get(0).state());
        assertEquals(1, shooter.feedCalls);
        assertEquals(300.0 + Math.hypot(24.0, 48.0), logic.lastRpm(), 1e-9);

        shooter.feeding = false;
        statuses.clear();
        logic.update(statuses);
        assertEquals(RequestStatus.State.DONE, statuses.get(0).state());
        assertEquals(1, turret.holdCalls);
    }

    private static TurretLogic turretLogic(RecordingTurret turret) {
        TurretLogic logic = new TurretLogic(turret);
        logic.update(new Pose(0.0, 0.0, 0.0));
        return logic;
    }

    private static final class RecordingShooter implements IShooter {
        boolean ready;
        boolean feeding;
        int feedCalls;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void spinUp(double rpm) {}
        @Override public void spinDown() {}
        @Override public boolean isReady() { return ready; }
        @Override public void feed() { feedCalls++; feeding = true; }
        @Override public boolean isFeeding() { return feeding; }
        @Override public void setHoodAngleDeg(double angleDeg) {}
        @Override public boolean hoodSettled() { return false; }
    }

    private static final class RecordingTurret implements ITurret {
        int holdCalls;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void setRobotPose(com.pedropathing.math.Pose pose) {}
        @Override public void aimAt(double fieldX, double fieldY) {}
        @Override public void scan() {}
        @Override public void hold() { holdCalls++; }
        @Override public boolean onTarget() { return true; }
        @Override public double angleRad() { return 0.0; }
        @Override public AimResult aimRelative(double angleRad) { return AimResult.ACCEPTED; }
        @Override public AimResult aimStatus() { return AimResult.ACCEPTED; }
        @Override public void disable() {}
    }
}
