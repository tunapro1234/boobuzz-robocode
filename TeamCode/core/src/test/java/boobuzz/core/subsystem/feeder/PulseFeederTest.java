package boobuzz.core.subsystem.feeder;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class PulseFeederTest {

    private static final long TICK_MS = 20;
    private static final long DELAY_MS = RobotConstants.FEEDER_POST_PULSE_DELAY_MS;

    @Test
    public void archiveTimingConstantsArePinned() {
        assertEquals(1.0, RobotConstants.FEEDER_PULSE_POWER, 0.0);
        assertEquals(350L, RobotConstants.FEEDER_PULSE_MS);
        assertEquals(100L, RobotConstants.FEEDER_POST_PULSE_DELAY_MS);
        assertEquals("BRAKE", RobotConstants.FEEDER.zeroPower());
    }

    @Test
    public void heldPulseAndDelayMatchesTwentyMillisecondFixture() {
        PulseFeeder feeder = new PulseFeeder();
        List<long[]> edges = new ArrayList<>();
        double last = 0.0;
        for (long t = 0; t <= 1000; t += TICK_MS) {
            feeder.requestPulseAndDelay(DELAY_MS);
            double power = tick(feeder, t);
            if (power != last) {
                edges.add(new long[] {t, (long) power});
                last = power;
            }
        }
        // Pulse on at 0, off at the first tick >= 350 ms (360), back on 100 ms later (460).
        assertEdge(edges.get(0), 0, 1);
        assertEdge(edges.get(1), 360, 0);
        assertEdge(edges.get(2), 460, 1);
        assertEdge(edges.get(3), 820, 0);
        assertEdge(edges.get(4), 920, 1);
        long pulseEnd = edges.get(1)[0] - edges.get(0)[0];
        assertTrue(pulseEnd >= 350 && pulseEnd < 370);
        long gap = edges.get(2)[0] - edges.get(1)[0];
        assertTrue(Math.abs(gap - DELAY_MS) <= TICK_MS);
    }

    @Test
    public void phasesFollowIdlePulsingGap() {
        PulseFeeder feeder = new PulseFeeder();
        assertEquals(PulseFeeder.Phase.IDLE, feeder.phase());
        feeder.requestPulseAndDelay(DELAY_MS);
        tick(feeder, 0);
        assertEquals(PulseFeeder.Phase.PULSING, feeder.phase());
        tick(feeder, 340);
        assertTrue(feeder.isPulsing());
        tick(feeder, 360);
        assertEquals(PulseFeeder.Phase.GAP, feeder.phase());
        assertEquals(DELAY_MS, feeder.delayRemainingMs());
        tick(feeder, 440);
        assertEquals(20L, feeder.delayRemainingMs());
        tick(feeder, 460);
        assertEquals(PulseFeeder.Phase.PULSING, feeder.phase());
    }

    @Test
    public void releaseMidPulseFinishesCurrentPulseOnly() {
        PulseFeeder feeder = new PulseFeeder();
        feeder.requestPulseAndDelay(DELAY_MS);
        assertEquals(1.0, tick(feeder, 0), 0.0);
        feeder.clearRequest();
        for (long t = TICK_MS; t < 360; t += TICK_MS) {
            assertEquals("pulse must finish at t=" + t, 1.0, tick(feeder, t), 0.0);
        }
        for (long t = 360; t <= 2000; t += TICK_MS) {
            assertEquals("no pulse after release at t=" + t, 0.0, tick(feeder, t), 0.0);
        }
        assertEquals(PulseFeeder.Phase.IDLE, feeder.phase());
    }

    @Test
    public void clearMidGapEndsGapWithoutNewPulse() {
        PulseFeeder feeder = new PulseFeeder();
        feeder.requestPulseAndDelay(DELAY_MS);
        runUntil(feeder, 0, 360);
        assertTrue(feeder.isInDelay());
        feeder.clearRequest();
        for (long t = 380; t <= 1500; t += TICK_MS) {
            assertEquals(0.0, tick(feeder, t), 0.0);
        }
        assertEquals(PulseFeeder.Phase.IDLE, feeder.phase());
    }

    @Test
    public void stopCancelsPulseAndGapImmediately() {
        PulseFeeder feeder = new PulseFeeder();
        feeder.requestPulseAndDelay(DELAY_MS);
        runUntil(feeder, 0, 100);
        feeder.stop();
        assertEquals(0.0, tick(feeder, 120), 0.0);
        assertEquals(PulseFeeder.Phase.IDLE, feeder.phase());

        feeder.requestPulseAndDelay(DELAY_MS);
        runUntil(feeder, 200, 580);
        assertTrue(feeder.isInDelay());
        feeder.stop();
        assertEquals(0.0, tick(feeder, 600), 0.0);
        assertEquals(PulseFeeder.Phase.IDLE, feeder.phase());
        for (long t = 620; t <= 1200; t += TICK_MS) {
            assertEquals(0.0, tick(feeder, t), 0.0);
        }
    }

    @Test
    public void restartAfterStopGivesFullFreshPulse() {
        PulseFeeder feeder = new PulseFeeder();
        feeder.requestPulseAndDelay(DELAY_MS);
        runUntil(feeder, 0, 300);
        feeder.stop();
        tick(feeder, 320);
        feeder.requestPulseAndDelay(DELAY_MS);
        feeder.clearRequest();
        feeder.requestPulseAndDelay(DELAY_MS);
        assertEquals(1.0, tick(feeder, 1000), 0.0);
        assertEquals(1.0, tick(feeder, 1340), 0.0);
        assertEquals(0.0, tick(feeder, 1360), 0.0);
    }

    @Test
    public void manualReverseRecoveryHoldsPowerWithoutPulseState() {
        PulseFeeder feeder = new PulseFeeder();
        feeder.requestPulseAndDelay(DELAY_MS);
        runUntil(feeder, 0, 100);
        feeder.stop();
        feeder.setPower(-1.0);
        for (long t = 120; t <= 1000; t += TICK_MS) {
            assertEquals(-1.0, tick(feeder, t), 0.0);
        }
        assertEquals(PulseFeeder.Phase.IDLE, feeder.phase());
        feeder.stop();
        assertEquals(0.0, tick(feeder, 1020), 0.0);
        feeder.setPower(Double.NaN);
        assertEquals(0.0, tick(feeder, 1040), 0.0);
    }

    @Test
    public void delayParameterNeverChangesPulseLength() {
        // Pins against an accidental 100-ms pulse or a revived 500-ms delay default.
        PulseFeeder feeder = new PulseFeeder();
        List<Long> onTicks = new ArrayList<>();
        List<Long> offTicks = new ArrayList<>();
        double last = 0.0;
        for (long t = 0; t <= 2000; t += TICK_MS) {
            feeder.requestPulseAndDelay(500);
            double power = tick(feeder, t);
            if (power != last) {
                (power > 0 ? onTicks : offTicks).add(t);
                last = power;
            }
        }
        assertEquals(360L, offTicks.get(0) - onTicks.get(0));
        assertEquals(500L, onTicks.get(1) - offTicks.get(0));
        assertEquals(360L, offTicks.get(1) - onTicks.get(1));

        PulseFeeder defaultFeeder = new PulseFeeder();
        defaultFeeder.requestPulseAndDelay(DELAY_MS);
        runUntil(defaultFeeder, 0, 360);
        assertEquals(DELAY_MS, defaultFeeder.delayRemainingMs());
    }

    @Test
    public void heldPlainRequestRestartsPulseWithoutStopping() {
        PulseFeeder feeder = new PulseFeeder();
        feeder.requestPulse();
        for (long t = 0; t <= 1000; t += TICK_MS) {
            assertEquals(1.0, tick(feeder, t), 0.0);
        }
        feeder.clearRequest();
        // Current pulse restarted at 720 ms, so it ends at the first tick >= 1070 ms.
        assertEquals(1.0, tick(feeder, 1060), 0.0);
        assertEquals(0.0, tick(feeder, 1080), 0.0);
        assertFalse(feeder.isRunning());
    }

    @Test
    public void negativeDelayIsRejected() {
        try {
            new PulseFeeder().requestPulseAndDelay(-1);
            fail("negative delay must throw");
        } catch (IllegalArgumentException expected) {
            // expected
        }
    }

    @Test
    public void writesOnlyTheFeederMotorAndNoEvents() {
        PulseFeeder feeder = new PulseFeeder();
        feeder.observe(state(0));
        RobotAction.Builder out = new RobotAction.Builder();
        feeder.update(out);
        RobotAction action = out.build();
        assertEquals(1, action.motors().size());
        assertEquals(0.0, action.motor(RobotConstants.FEEDER_MOTOR_NAME), 0.0);
        assertTrue(action.events().isEmpty());
    }

    private static void runUntil(PulseFeeder feeder, long fromMs, long toMs) {
        for (long t = fromMs; t <= toMs; t += TICK_MS) {
            tick(feeder, t);
        }
    }

    private static double tick(PulseFeeder feeder, long t) {
        feeder.observe(state(t));
        RobotAction.Builder out = new RobotAction.Builder();
        feeder.update(out);
        return out.build().motor(RobotConstants.FEEDER_MOTOR_NAME);
    }

    private static RobotState state(long t) {
        return new RobotState(t, Map.of(), Map.of(), 0.0, new Pose(0.0, 0.0, 0.0), 12.0);
    }

    private static void assertEdge(long[] edge, long t, long power) {
        assertEquals("edge time", t, edge[0]);
        assertEquals("edge power at " + t, power, edge[1]);
    }
}
