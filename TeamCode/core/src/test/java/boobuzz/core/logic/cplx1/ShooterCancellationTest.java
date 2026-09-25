package boobuzz.core.logic.cplx1;

import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.logic.cplx1.CplxEngine1;
import boobuzz.core.subsystem.IDrive;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.stub.StubIntake;
import boobuzz.core.subsystem.IShooter;
import boobuzz.core.subsystem.ITurret;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ShooterCancellationTest {

    @Test
    public void cancelActiveShotStopsShooterAndEmitsTerminalStatus() {
        RecordingShooter shooter = new RecordingShooter();
        ShooterLogic logic = new ShooterLogic(shooter, turret());
        logic.observe(new Pose(24.0, 48.0, 0.0));
        logic.requestShot(41, 2);
        List<RequestStatus> statuses = new ArrayList<>();

        assertTrue(logic.cancel(41, statuses));
        assertEquals(1, shooter.spinDownCalls);
        assertEquals(RequestStatus.State.REJECTED, statuses.get(0).state());
        assertEquals(41, statuses.get(0).id());
        assertEquals(ShooterLogic.State.IDLE, logic.state());
    }

    @Test
    public void cancelActiveSpinUpStopsShooterAndEmitsTerminalStatus() {
        RecordingShooter shooter = new RecordingShooter();
        ShooterLogic logic = new ShooterLogic(shooter, turret());
        logic.requestSpinUp(42, 400.0);
        List<RequestStatus> statuses = new ArrayList<>();

        assertTrue(logic.cancel(42, statuses));
        assertEquals(1, shooter.spinDownCalls);
        assertEquals(RequestStatus.State.REJECTED, statuses.get(0).state());
        assertEquals(42, statuses.get(0).id());
        assertEquals(ShooterLogic.State.IDLE, logic.state());
    }

    @Test
    public void cplxEngineRoutesPerRequestCancelToShooterOwner() {
        RecordingShooter shooter = new RecordingShooter();
        CplxEngine1 engine = new CplxEngine1(new Subsystems(
                new RecordingDrive(), shooter, new StubIntake()));
        engine.sense(state(0L));
        engine.act(RequestBatch.of(Request.spinUp(51, 400.0)));
        engine.drainStatuses();

        engine.sense(state(20L));
        engine.act(new RequestBatch(null, List.of(), new int[] {51}));

        assertEquals(1, shooter.spinDownCalls);
        assertTrue(engine.drainStatuses().stream()
                .anyMatch(status -> status.id() == 51
                        && status.state() == RequestStatus.State.REJECTED));
    }

    private static TurretLogic turret() {
        return new TurretLogic(new ITurret() {
            @Override public void observe(RobotState state) {}
            @Override public void update(RobotAction.Builder out) {}
            @Override public void aimAt(double fieldX, double fieldY) {}
            @Override public void scan() {}
            @Override public void hold() {}
            @Override public boolean onTarget() { return true; }
            @Override public double angleRad() { return 0.0; }
        });
    }

    private static RobotState state(long t) {
        return new RobotState(t, Map.of(), Map.of(), 0.0,
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
        int spinDownCalls;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void spinUp(double rpm) {}
        @Override public void spinDown() { spinDownCalls++; }
        @Override public boolean isReady() { return false; }
        @Override public void feed() {}
        @Override public boolean isFeeding() { return false; }
        @Override public void setHoodAngleDeg(double angleDeg) {}
        @Override public boolean hoodSettled() { return false; }
    }
}
