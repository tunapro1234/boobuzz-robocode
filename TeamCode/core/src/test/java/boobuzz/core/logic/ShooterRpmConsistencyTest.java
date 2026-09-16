package boobuzz.core.logic;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.logic.cplx1.CplxEngine1;
import boobuzz.core.logic.direct.DirectEngine;
import boobuzz.core.subsystem.IDrive;
import boobuzz.core.subsystem.IShooter;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.stub.StubIntake;
import boobuzz.core.subsystem.stub.StubTurret;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ShooterRpmConsistencyTest {

    @Test
    public void countOnlyShotsUseTheSameCalibratedRpm() {
        RecordingShooter cplxShooter = new RecordingShooter();
        CplxEngine1 cplx = new CplxEngine1(new Subsystems(
                new RecordingDrive(), cplxShooter, new StubIntake(), new StubTurret()));
        cplx.sense(state());
        cplx.act(RequestBatch.of(Request.shoot(1, 2)));

        RecordingShooter directShooter = new RecordingShooter();
        DirectEngine direct = new DirectEngine(new Subsystems(
                new RecordingDrive(), directShooter, new StubIntake(), new StubTurret()));
        direct.sense(state());
        direct.act(RequestBatch.of(Request.shoot(2, 2)));

        assertEquals(cplxShooter.rpm, directShooter.rpm, 1e-9);
        assertEquals(300.0 + Math.hypot(24.0, 48.0), cplxShooter.rpm, 1e-9);
    }

    @Test
    public void explicitShotsUseTheSameValidatedRpm() {
        RecordingShooter cplxShooter = new RecordingShooter();
        CplxEngine1 cplx = new CplxEngine1(new Subsystems(
                new RecordingDrive(), cplxShooter, new StubIntake(), new StubTurret()));
        cplx.sense(state());
        cplx.act(RequestBatch.of(Request.shoot(3, 1, 425.0)));

        RecordingShooter directShooter = new RecordingShooter();
        DirectEngine direct = new DirectEngine(new Subsystems(
                new RecordingDrive(), directShooter, new StubIntake(), new StubTurret()));
        direct.sense(state());
        direct.act(RequestBatch.of(Request.shoot(4, 1, 425.0)));

        assertEquals(425.0, cplxShooter.rpm, 1e-9);
        assertEquals(cplxShooter.rpm, directShooter.rpm, 1e-9);
    }

    @Test
    public void missingCountAndInvalidExplicitRpmAreRejected() {
        DirectEngine direct = new DirectEngine(new Subsystems(
                new RecordingDrive(), new RecordingShooter(), new StubIntake(), new StubTurret()));
        direct.sense(state());
        direct.act(RequestBatch.of(Request.of(5, boobuzz.core.contract.RequestType.SHOOT)));
        assertEquals(RequestStatus.State.REJECTED, direct.drainStatuses().get(0).state());

        CplxEngine1 cplx = new CplxEngine1(new Subsystems(
                new RecordingDrive(), new RecordingShooter(), new StubIntake(), new StubTurret()));
        cplx.sense(state());
        cplx.act(RequestBatch.of(Request.shoot(6, 1, Double.NaN)));
        assertTrue(cplx.drainStatuses().stream()
                .anyMatch(status -> status.state() == RequestStatus.State.REJECTED));
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
        double rpm;

        @Override public void observe(RobotState state) {}
        @Override public void update(RobotAction.Builder out) {}
        @Override public void spinUp(double rpm) { this.rpm = rpm; }
        @Override public void spinDown() { rpm = 0.0; }
        @Override public boolean isReady() { return false; }
        @Override public void feed() {}
        @Override public boolean isFeeding() { return false; }
    }
}
