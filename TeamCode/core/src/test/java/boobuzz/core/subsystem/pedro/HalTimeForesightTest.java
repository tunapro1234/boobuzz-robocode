package boobuzz.core.subsystem.pedro;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.Mechanism;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

/** Pedro's end-of-path hold timeout must follow HAL time, never the wall clock. */
public class HalTimeForesightTest {

    private static final long DT_MS = 20L;
    private static final long END_MS = 300L;
    /** Hold is first calculated here: t=20 follow, t=40 parametric end, t=60 hold(). */
    private static final long HOLD_ARMED_MS = 80L;
    /** Pedro core 3.0.0 ForesightConfig.timeoutConstraint default (javap: ldc 100.0d). */
    private static final double PEDRO_DEFAULT_TIMEOUT_MS = 100.0;

    @Test
    public void pedroDefaultTimeoutIsKept() {
        ForesightConfig config = PedroConstants.createForesightConfig(Mechanism.DEFAULT);
        assertEquals(PEDRO_DEFAULT_TIMEOUT_MS, config.timeoutConstraint.get(), 0.0);
    }

    @Test
    public void holdTimeoutFiresOnHalTimeNotWallClock() throws InterruptedException {
        List<String> fast = run(-1L);
        // A slow tick right after the hold starts would end Pedro's wall-clock hold
        // timeout at once; on HAL time it must change nothing.
        List<String> slow = run(HOLD_ARMED_MS + DT_MS);
        assertEquals(fast, slow);

        long firstDone = -1L;
        for (String tick : fast) {
            long t = Long.parseLong(tick.substring(2, tick.indexOf(' ')));
            // Before the request (t < 20) an idle Foresight reports not busy.
            if (t >= 20L && tick.contains("done=true")) {
                firstDone = t;
                break;
            }
        }
        // Heading error keeps the hold busy, so only the timeout can end it: the first
        // tick whose HAL time is strictly more than 100 ms after the hold armed.
        assertEquals(HOLD_ARMED_MS + 120L, firstDone);
    }

    @Test
    public void configuredControllersDoNotReadTheWallClock() {
        ForesightConfig config = PedroConstants.createForesightConfig(Mechanism.DEFAULT);
        List<Controller> controllers = List.of(
                config.headingFeedback.get(), config.headingStaticFF.get(),
                config.forwardTranslational.get(), config.strafeTranslational.get(),
                config.brake.get(), config.coast.get());
        for (Controller controller : controllers) {
            // Both classes measure dt with System.nanoTime (javap, Pedro core 3.0.0).
            assertFalse("wall-clock controller: " + controller,
                    controller instanceof Controller.TimedController
                            || controller instanceof PIDController);
        }
    }

    @Test
    public void holdTimeoutReArmsWhenHalTimeGoesBackwards() {
        AtomicLong now = new AtomicLong(1_000L);
        HalTimeForesight foresight = new HalTimeForesight(
                PedroConstants.createForesightConfig(Mechanism.DEFAULT), now::get);
        HalDrivetrain drivetrain = new HalDrivetrain(PedroDrive.wheelNames(Mechanism.DEFAULT));
        // Robot at the target point but 0.5 rad off heading: only the timeout ends it.
        Pose target = new Pose(10.0, 10.0, 0.5);
        MotionState state = MotionState.ofVelocity(new Pose(10.0, 10.0, 0.0), Velocity.zero());

        foresight.reset();
        foresight.calculateHold(drivetrain, target, state, false, DT_MS / 1000.0);  // arms at 1000
        long firstDone = -1L;
        for (long t = 0L; t <= 400L && firstDone < 0L; t += DT_MS) {
            now.set(t);   // time base restarted below the armed start
            foresight.calculateHold(drivetrain, target, state, false, DT_MS / 1000.0);
            if (!foresight.isBusy()) {
                firstDone = t;
            }
        }
        // Re-armed at t=0: first tick strictly more than 100 ms later. Without the
        // guard the hold would stay busy until t > 1100.
        assertEquals(120L, firstDone);
    }

    /** Returns one line per tick; sleeps before the tick at {@code slowTickMs}. */
    private static List<String> run(long slowTickMs) throws InterruptedException {
        PedroDrive drive = new PedroDrive(Mechanism.DEFAULT, new PathRegistry());
        List<String> ticks = new ArrayList<>();
        for (long t = 0L; t <= END_MS; t += DT_MS) {
            if (t == slowTickMs) {
                Thread.sleep(150L);
            }
            // At the end point from t=40 on, but 0.5 rad off heading.
            Pose pose = t < 40L ? new Pose(0.0, 0.0, 0.0) : new Pose(24.0, 0.0, 0.5);
            drive.observe(new RobotState(t, Map.of(), Map.of(), pose.heading(), pose, 12.0));
            if (t == 20L) {
                drive.follow(PathRequest.goTo(new Pose(24.0, 0.0, 0.0),
                        PathRequest.Constraints.defaults()));
            }
            RobotAction.Builder out = new RobotAction.Builder();
            drive.update(out);
            ticks.add("t=" + t + " done=" + drive.pathDone() + " " + out.build());
        }
        return ticks;
    }
}
