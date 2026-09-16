package boobuzz.core.logic.direct_engine;

import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Intent;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.subsystem.StubIntake;
import boobuzz.core.subsystem.StubShooter;
import boobuzz.core.subsystem.Subsystems;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class DirectEngineTest {

    @Test
    public void manualIntentPassesThroughDrive() {
        RecordingDrive drive = new RecordingDrive();
        DirectEngine engine = new DirectEngine(
                new Subsystems(drive, new StubShooter(), new StubIntake()));
        engine.sense(state(0));
        engine.act(Intent.of(new Drive.Manual(0.2, -0.3, 0.4)));

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

        cycle(engine, 0, new Intent(Drive.HOLD,
                java.util.List.of(Request.of(id, boobuzz.core.contract.RequestType.SHOOT,
                        2.0, 300.0)), new int[0]));
        assertEquals(RequestStatus.State.ACTIVE, status(engine).state());

        cycle(engine, spinupMs, Intent.idle());
        assertEquals("shooter.feed.start", engine.action().events().get(0).name());
        assertEquals(RequestStatus.State.ACTIVE, status(engine).state());

        cycle(engine, spinupMs + feedMs, Intent.idle());
        assertTrue(engine.action().events().stream()
                .anyMatch(event -> event.name().equals("shooter.feed.start")));
        assertEquals(RequestStatus.State.ACTIVE, status(engine).state());

        cycle(engine, spinupMs + 2L * feedMs, Intent.idle());
        assertTrue(engine.action().events().stream()
                .anyMatch(event -> event.name().equals("shooter.feed.end")));
        assertEquals(RequestStatus.State.DONE, status(engine).state());
    }

    @Test
    public void intakeRequestsAppearAsActionEvents() {
        DirectEngine engine = engine();
        cycle(engine, 20, new Intent(Drive.HOLD,
                java.util.List.of(Request.of(1, boobuzz.core.contract.RequestType.INTAKE_ON, 0.8)),
                new int[0]));
        assertEquals("intake.on", engine.action().events().get(0).name());

        cycle(engine, 40, new Intent(Drive.HOLD,
                java.util.List.of(Request.of(2, boobuzz.core.contract.RequestType.INTAKE_OFF)),
                new int[0]));
        assertEquals("intake.off", engine.action().events().get(0).name());
    }

    private static DirectEngine engine() {
        return new DirectEngine(new Subsystems(
                new RecordingDrive(), new StubShooter(), new StubIntake()));
    }

    private static void cycle(DirectEngine engine, long tMs, Intent intent) {
        engine.sense(state(tMs));
        engine.act(intent);
    }

    private static RequestStatus status(DirectEngine engine) {
        return engine.drainStatuses().get(0);
    }

    private static RobotState state(long tMs) {
        return new RobotState(tMs, Map.of(), Map.of(), 0.0,
                new Pose(0.0, 0.0, 0.0), 12.6);
    }

    private static final class RecordingDrive implements boobuzz.core.subsystem.Drive {
        private double vx;
        private double vy;
        private double omega;
        private boolean stopped;

        @Override public void observe(RobotState state) {}

        @Override public void update(RobotAction.Builder out) {
            if (stopped) {
                return;
            }
            out.motor("fl", vx);
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
