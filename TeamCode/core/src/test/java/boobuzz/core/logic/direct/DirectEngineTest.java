package boobuzz.core.logic.direct;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RequestType;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.subsystem.IDrive;
import boobuzz.core.subsystem.stub.StubIntake;
import boobuzz.core.subsystem.stub.StubShooter;
import boobuzz.core.subsystem.Subsystems;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class DirectEngineTest {

    @Test
    public void manualStreamPassesThroughDrive() {
        RecordingDrive drive = new RecordingDrive();
        DirectEngine engine = new DirectEngine(
                new Subsystems(drive, new StubShooter(), new StubIntake()));
        engine.sense(state(0));
        engine.act(new RequestBatch(RequestStream.manual(0.2, -0.3, 0.4), List.of()));

        assertEquals(0.2, drive.vx, 1e-9);
        assertEquals(-0.3, drive.vy, 1e-9);
        assertEquals(0.4, drive.omega, 1e-9);
        assertEquals(0.2, engine.action().motor("fl"), 1e-9);
    }

    @Test
    public void shootFeedsTwiceThenReportsDone() {
        DirectEngine engine = engine();
        int id = 42;
        long spinupMs = Math.round(boobuzz.core.hal.RobotConstants.STUB_SPINUP_S * 1000.0);
        long feedMs = Math.round(boobuzz.core.hal.RobotConstants.STUB_FEED_S * 1000.0);

        cycle(engine, 0, batch(Request.of(id, RequestType.SHOOT, 2.0, 300.0)));
        assertEquals(RequestStatus.State.ACTIVE, status(engine).state());

        cycle(engine, spinupMs, RequestBatch.idle());
        assertEquals("shooter.feed.start", engine.action().events().get(0).name());
        assertEquals(RequestStatus.State.ACTIVE, status(engine).state());

        cycle(engine, spinupMs + feedMs, RequestBatch.idle());
        assertTrue(engine.action().events().stream()
                .anyMatch(event -> event.name().equals("shooter.feed.start")));
        assertEquals(RequestStatus.State.ACTIVE, status(engine).state());

        cycle(engine, spinupMs + 2L * feedMs, RequestBatch.idle());
        assertTrue(engine.action().events().stream()
                .anyMatch(event -> event.name().equals("shooter.feed.end")));
        assertEquals(RequestStatus.State.DONE, status(engine).state());
    }

    @Test
    public void intakeRequestsAppearAsActionEvents() {
        DirectEngine engine = engine();
        cycle(engine, 20, batch(Request.of(1, RequestType.INTAKE_ON, 0.8)));
        assertEquals("intake.on", engine.action().events().get(0).name());

        cycle(engine, 40, batch(Request.of(2, RequestType.INTAKE_OFF)));
        assertEquals("intake.off", engine.action().events().get(0).name());
    }

    @Test
    public void manualStreamRejectsActivePath() {
        RecordingDrive drive = new RecordingDrive();
        DirectEngine engine = new DirectEngine(
                new Subsystems(drive, new StubShooter(), new StubIntake()));
        cycle(engine, 0, batch(Request.path(7, PathRequest.named("test-line"))));
        engine.drainStatuses();

        cycle(engine, 20, new RequestBatch(RequestStream.manual(1, 0, 0), List.of()));
        RequestStatus rejected = engine.drainStatuses().stream()
                .filter(status -> status.id() == 7).findFirst().orElseThrow();
        assertEquals(RequestStatus.State.REJECTED, rejected.state());
        assertEquals("overridden by manual drive", rejected.note());
        assertEquals(1.0, drive.vx, 1e-9);
    }

    private static RequestBatch batch(Request request) {
        return new RequestBatch(RequestStream.idle(), List.of(request));
    }

    private static DirectEngine engine() {
        return new DirectEngine(new Subsystems(
                new RecordingDrive(), new StubShooter(), new StubIntake()));
    }

    private static void cycle(DirectEngine engine, long tMs, RequestBatch batch) {
        engine.sense(state(tMs));
        engine.act(batch);
    }

    private static RequestStatus status(DirectEngine engine) {
        return engine.drainStatuses().get(0);
    }

    private static RobotState state(long tMs) {
        return new RobotState(tMs, Map.of(), Map.of(), 0.0,
                new Pose(0.0, 0.0, 0.0), 12.6);
    }

    private static final class RecordingDrive implements IDrive {
        private double vx;
        private double vy;
        private double omega;
        private boolean stopped;

        @Override public void observe(RobotState state) {}

        @Override public void update(RobotAction.Builder out) {
            if (!stopped) {
                out.motor("fl", vx);
            }
        }

        @Override public void manual(double vx, double vy, double omega) {
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
            stopped = false;
        }

        @Override public void follow(PathRequest request) { stopped = false; }

        @Override public void stop() { stopped = true; }

        @Override public boolean pathDone() { return true; }

        @Override public Pose pose() { return new Pose(0.0, 0.0, 0.0); }
    }
}
