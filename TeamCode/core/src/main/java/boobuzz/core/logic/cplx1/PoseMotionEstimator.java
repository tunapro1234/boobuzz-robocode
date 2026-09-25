package boobuzz.core.logic.cplx1;

import boobuzz.core.hal.RobotConstants;

import com.pedropathing.math.Pose;

import java.util.ArrayDeque;
import java.util.Iterator;

/**
 * Estimated chassis motion from the Pinpoint poses already in RobotState: a fixed-window
 * difference, not a wheel-speed sensor. Any gap, time reset or explicit localization
 * reset invalidates the stationary gate until a full window is rebuilt.
 */
public final class PoseMotionEstimator {

    private final ArrayDeque<double[]> samples = new ArrayDeque<>();
    private long lastT;
    private double lastRawHeading;
    private double unwrappedHeading;
    private boolean valid;
    private double speedInS = Double.NaN;
    private double yawRateDegS = Double.NaN;
    private boolean still;
    private long stillSinceMs;

    /** Feed one sample per tick; a null or non-finite pose invalidates the history. */
    public void observe(long tMs, Pose pose) {
        if (pose == null || !Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading())) {
            reset();
            return;
        }
        if (!samples.isEmpty()) {
            long gap = tMs - lastT;
            if (gap <= 0 || gap > RobotConstants.STATIONARY_MAX_SAMPLE_GAP_MS) {
                reset();
            }
        }
        if (samples.isEmpty()) {
            unwrappedHeading = pose.heading();
        } else {
            unwrappedHeading += wrap(pose.heading() - lastRawHeading);
        }
        lastRawHeading = pose.heading();
        lastT = tMs;
        samples.addLast(new double[] {tMs, pose.x(), pose.y(), unwrappedHeading});
        evaluate(tMs);
    }

    /** Explicit localization reset (or any pose discontinuity the caller knows about). */
    public void reset() {
        samples.clear();
        valid = false;
        still = false;
        speedInS = Double.NaN;
        yawRateDegS = Double.NaN;
    }

    public boolean valid() {
        return valid;
    }

    public double speedInS() {
        return speedInS;
    }

    public double yawRateDegS() {
        return yawRateDegS;
    }

    /** True once speed and yaw rate have stayed inside the gate for the hold time. */
    public boolean stationary(long nowMs) {
        return valid && still && nowMs - stillSinceMs >= RobotConstants.STATIONARY_HOLD_MS;
    }

    private void evaluate(long now) {
        long windowStart = now - RobotConstants.STATIONARY_WINDOW_MS;
        // Keep exactly one sample at or before the window start.
        while (samples.size() >= 2) {
            Iterator<double[]> it = samples.iterator();
            it.next();
            if (it.next()[0] <= windowStart) {
                samples.removeFirst();
            } else {
                break;
            }
        }
        double[] oldest = samples.peekFirst();
        if (oldest == null || oldest[0] > windowStart) {
            valid = false;
            still = false;
            return;
        }
        double[] newest = samples.peekLast();
        double dt = (newest[0] - oldest[0]) / 1000.0;
        speedInS = Math.hypot(newest[1] - oldest[1], newest[2] - oldest[2]) / dt;
        yawRateDegS = Math.toDegrees(Math.abs(newest[3] - oldest[3])) / dt;
        valid = true;
        boolean inGate = speedInS <= RobotConstants.STATIONARY_MAX_SPEED_IN_S
                && yawRateDegS <= RobotConstants.STATIONARY_MAX_YAW_DEG_S;
        if (inGate && !still) {
            stillSinceMs = now;
        }
        still = inGate;
    }

    private static double wrap(double rad) {
        return Math.atan2(Math.sin(rad), Math.cos(rad));
    }
}
