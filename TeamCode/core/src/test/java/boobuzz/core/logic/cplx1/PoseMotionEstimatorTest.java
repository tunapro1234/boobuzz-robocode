package boobuzz.core.logic.cplx1;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class PoseMotionEstimatorTest {

    private static final long DT = 20;

    @Test
    public void missingHistoryIsNotStationary() {
        PoseMotionEstimator est = new PoseMotionEstimator();
        assertFalse(est.stationary(0));
        for (long t = 0; t < 200; t += DT) {
            est.observe(t, new Pose(10, 10, 0));
            assertFalse("window not full at " + t, est.valid());
        }
        est.observe(200, new Pose(10, 10, 0));
        assertTrue(est.valid());
        assertFalse("hold time not yet elapsed", est.stationary(200));
    }

    @Test
    public void stationaryAfterWindowPlusHold() {
        PoseMotionEstimator est = new PoseMotionEstimator();
        long firstStationary = -1;
        for (long t = 0; t <= 600; t += DT) {
            est.observe(t, new Pose(10, 10, 1.0));
            if (firstStationary < 0 && est.stationary(t)) {
                firstStationary = t;
            }
        }
        // Window valid at 200 ms, then 150 ms hold; first 20 ms tick at or after 350 ms.
        assertEquals(360, firstStationary);
        assertEquals(0.0, est.speedInS(), 1e-12);
    }

    @Test
    public void constantVelocityMeasuresSpeedAndBlocksGate() {
        PoseMotionEstimator est = new PoseMotionEstimator();
        for (long t = 0; t <= 1000; t += DT) {
            est.observe(t, new Pose(10 + 3.0 * t / 1000.0, 10, 0));
            assertFalse(est.stationary(t));
        }
        assertEquals(3.0, est.speedInS(), 1e-9);
    }

    @Test
    public void slowCreepInsideGateCountsAsStationary() {
        PoseMotionEstimator est = new PoseMotionEstimator();
        for (long t = 0; t <= 1000; t += DT) {
            est.observe(t, new Pose(10 + 1.5 * t / 1000.0, 10, 0));
        }
        assertEquals(1.5, est.speedInS(), 1e-9);
        assertTrue(est.stationary(1000));
    }

    @Test
    public void yawRateUsesUnwrappedHeadingAcrossPi() {
        PoseMotionEstimator est = new PoseMotionEstimator();
        double rate = Math.toRadians(10.0); // 10 deg/s, crossing +pi
        for (long t = 0; t <= 1000; t += DT) {
            double h = Math.PI - 0.05 + rate * t / 1000.0;
            est.observe(t, new Pose(10, 10, Math.atan2(Math.sin(h), Math.cos(h))));
            assertFalse(est.stationary(t));
        }
        assertEquals(10.0, est.yawRateDegS(), 1e-6);
    }

    @Test
    public void pinpointNoiseStaysStationary() {
        // Simulator Pinpoint noise: gaussian sigma 0.05 in (x, y) and 0.002 rad (heading).
        Random random = new Random(42);
        PoseMotionEstimator est = new PoseMotionEstimator();
        for (long t = 0; t <= 30_000; t += DT) {
            est.observe(t, new Pose(10 + 0.05 * random.nextGaussian(),
                    10 + 0.05 * random.nextGaussian(), 0.002 * random.nextGaussian()));
            if (t >= 360) {
                assertTrue("noise broke the gate at " + t, est.stationary(t));
            }
        }
    }

    @Test
    public void sampleGapResetsHistory() {
        PoseMotionEstimator est = stillFor(600);
        assertTrue(est.stationary(600));
        est.observe(720, new Pose(10, 10, 0)); // 120 ms gap > 100 ms
        assertFalse(est.valid());
        assertFalse(est.stationary(720));
    }

    @Test
    public void nonMonotonicTimeResetsHistory() {
        PoseMotionEstimator est = stillFor(600);
        est.observe(600, new Pose(10, 10, 0));
        assertFalse(est.valid());
        est.observe(100, new Pose(10, 10, 0));
        assertFalse(est.valid());
    }

    @Test
    public void poseJumpBlocksGateUntilWindowPasses() {
        PoseMotionEstimator est = stillFor(600);
        est.observe(620, new Pose(20, 10, 0)); // relocalization-style jump
        assertFalse(est.stationary(620));
        long t = 620;
        while (!est.stationary(t)) {
            t += DT;
            est.observe(t, new Pose(20, 10, 0));
        }
        // The jump sample anchors a clean window at 820 ms; hold ends at 970, next tick 980.
        assertEquals(980, t);
    }

    @Test
    public void explicitResetAndNullPoseInvalidate() {
        PoseMotionEstimator est = stillFor(600);
        est.reset();
        assertFalse(est.stationary(600));
        est = stillFor(600);
        est.observe(620, null);
        assertFalse(est.valid());
        assertFalse(est.stationary(620));
    }

    private static PoseMotionEstimator stillFor(long until) {
        PoseMotionEstimator est = new PoseMotionEstimator();
        for (long t = 0; t <= until; t += DT) {
            est.observe(t, new Pose(10, 10, 0));
        }
        return est;
    }
}
