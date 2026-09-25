package boobuzz.core.logic;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.logic.cplx1.CplxEngine1;
import boobuzz.core.logic.direct.DirectEngine;
import boobuzz.core.subsystem.IDrive;
import boobuzz.core.subsystem.IIntake;
import boobuzz.core.subsystem.IShooter;
import boobuzz.core.subsystem.Subsystems;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertTrue;

public class ManualOverrideSafetyTest {

    @Test
    public void cplxManualOverrideStopsAllSequenceOwners() {
        RecordingDrive drive = new RecordingDrive();
        RecordingShooter shooter = new RecordingShooter();
        RecordingIntake intake = new RecordingIntake();
        CplxEngine1 engine = new CplxEngine1(new Subsystems(drive, shooter, intake));
        engine.sense(state(0L));
        engine.act(attachedBatch());
        engine.drainStatuses();

        engine.sense(state(20L));
        engine.act(manualCancellation());

        assertTrue(drive.stopCalls > 0);
        assertTrue(shooter.spinDownCalls > 0);
        assertTrue(intake.stopCalls > 0);
    }

    @Test
    public void directManualOverrideStopsAllSequenceOwners() {
        RecordingDrive drive = new RecordingDrive();
        RecordingShooter shooter = new RecordingShooter();
        RecordingIntake intake = new RecordingIntake();
        DirectEngine engine = new DirectEngine(new Subsystems(drive, shooter, intake));
        engine.sense(state(0L));
        engine.act(attachedBatch());
        engine.drainStatuses();

        engine.sense(state(20L));
        engine.act(manualCancellation());

        assertTrue(drive.stopCalls > 0);
        assertTrue(shooter.spinDownCalls > 0);
        assertTrue(intake.stopCalls > 0);
    }

    private static RequestBatch attachedBatch() {
        return new RequestBatch(RequestStream.idle(), java.util.List.of(
                Request.path(1, PathRequest.named("test-line")),
                Request.intakeOn(2, 1.0),
                Request.spinUp(3, 400.0)));
    }

    private static RequestBatch manualCancellation() {
        return new RequestBatch(RequestStream.manual(0.5, 0.0, 0.0),
                java.util.List.of(), new int[] {1, 2, 3});
    }

    private static RobotState state(long t) {
        return new RobotState(t, Map.of(), Map.of(), 0.0,
                new Pose(24.0, 48.0, 0.0), 12.0);
    }

    private static final class RecordingDrive implements IDrive {
        int stopCalls;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void manual(double vx, double vy, double omega) {}
        @Override public void follow(PathRequest request) {}
        @Override public void stop() { stopCalls++; }
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
}
