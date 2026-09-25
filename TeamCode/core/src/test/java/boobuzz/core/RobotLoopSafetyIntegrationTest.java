package boobuzz.core;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.controller.IController;
import boobuzz.core.logic.cplx1.CplxEngine1;
import boobuzz.core.logic.direct.DirectEngine;
import boobuzz.core.subsystem.IDrive;
import boobuzz.core.subsystem.IIntake;
import boobuzz.core.subsystem.IShooter;
import boobuzz.core.subsystem.ITurret;
import boobuzz.core.subsystem.Subsystems;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Integration coverage for the R3 safety paths across controller, engines, and HAL. */
public class RobotLoopSafetyIntegrationTest {

    @Test
    public void manualTakeoverCancelsOwnersBeforeEngineHandoff() {
        RecordingHal hal = new RecordingHal();
        RecordingDrive drive = new RecordingDrive();
        RecordingShooter shooter = new RecordingShooter();
        RecordingIntake intake = new RecordingIntake();
        RecordingTurret turret = new RecordingTurret();
        Subsystems subsystems = new Subsystems(drive, shooter, intake, turret);
        CplxEngine1 cplx = new CplxEngine1(subsystems);
        DirectEngine direct = new DirectEngine(subsystems);
        List<RequestStatus> feedbackStatuses = new ArrayList<>();
        IController controller = new IController() {
            private int tick;

            @Override public RequestBatch decide(Feedback feedback) {
                feedbackStatuses.addAll(feedback.statuses());
                return switch (tick++) {
                    case 0 -> attachedRequests();
                    case 1 -> manualTakeover();
                    case 2 -> RequestBatch.of(Request.switchEngine(4, 1));
                    default -> RequestBatch.idle();
                };
            }
        };
        RobotLoop loop = new RobotLoop(hal, List.of(cplx, direct), cplx, controller);

        loop.tick();
        loop.tick();
        loop.tick();
        loop.tick();

        assertTrue(drive.stopCalls > 0);
        assertTrue(shooter.spinDownCalls > 0);
        assertTrue(intake.stopCalls > 0);
        assertTrue(turret.disableCalls > 0);
        assertTrue(feedbackStatuses.stream().anyMatch(status -> status.id() == 1
                && status.state() == RequestStatus.State.REJECTED));
        assertTrue(feedbackStatuses.stream().anyMatch(status -> status.id() == 2
                && status.state() == RequestStatus.State.DONE));
        assertTrue(feedbackStatuses.stream().noneMatch(status -> status.id() == 2
                && status.state() != RequestStatus.State.DONE));
        assertTrue(feedbackStatuses.stream().anyMatch(status -> status.id() == 3
                && status.state() == RequestStatus.State.REJECTED));
        assertEquals(0.0, hal.writes.get(2).motor("fl"), 1e-9);
    }

    private static RequestBatch attachedRequests() {
        return new RequestBatch(RequestStream.idle(), List.of(
                Request.path(1, PathRequest.named("test-line")),
                Request.intakeOn(2, 1.0),
                Request.spinUp(3, 400.0)));
    }

    private static RequestBatch manualTakeover() {
        return new RequestBatch(RequestStream.manual(0.5, 0.0, 0.0),
                List.of(), new int[] {1, 2, 3});
    }

    private static final class RecordingHal implements boobuzz.core.hal.IHal {
        long now;
        final List<RobotAction> writes = new ArrayList<>();

        @Override public long now() { return now; }
        @Override public RobotState read() {
            return new RobotState(now, Map.of(), Map.of(), 0.0,
                    new Pose(24.0, 48.0, 0.0), 12.0);
        }
        @Override public void write(RobotAction action) {
            writes.add(action);
            now += 20L;
        }
        @Override public boobuzz.core.contract.GamepadState get() {
            return boobuzz.core.contract.GamepadState.neutral();
        }
    }

    private static final class RecordingDrive implements IDrive {
        int stopCalls;
        double manualVx;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) { out.motor("fl", manualVx); }
        @Override public void manual(double vx, double vy, double omega) { manualVx = vx; }
        @Override public void follow(PathRequest request) {}
        @Override public void stop() { stopCalls++; manualVx = 0.0; }
        @Override public boolean pathDone() { return false; }
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
        @Override public void runOpenLoop(double power) {}
        @Override public void setFeederPower(double power) {}
        @Override public boolean isStopped() { return true; }
    }

    private static final class RecordingIntake implements IIntake {
        int stopCalls;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void run(double power) {}
        @Override public void stop() { stopCalls++; }
        @Override public boolean hasBall() { return false; }
    }

    private static final class RecordingTurret implements ITurret {
        int holdCalls;
        int disableCalls;

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
        @Override public void disable() { disableCalls++; }
    }
}
