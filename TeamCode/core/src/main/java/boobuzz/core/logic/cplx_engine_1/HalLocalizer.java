package boobuzz.core.logic.cplx_engine_1;

import boobuzz.core.contract.RobotState;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import java.util.Objects;

/**
 * Connects Pedro's localizer to HAL Pinpoint readings.
 *
 * <p>{@link #feed(RobotState)} is called every robot tick before the follower is
 * updated. {@link #update()} is a no-op; it does not read hardware or advance time.
 * Because {@link #reset()} cannot write Pinpoint, it only clears the software pose
 * offset and starts the next velocity calculation with zero speed like a first sample.
 */
public final class HalLocalizer implements Localizer {

    private MotionState motionState = MotionState.zero();
    private Pose rawPose;
    private Pose previousRawPose;
    private long previousTimeMs;
    private boolean hasPreviousSample;

    private double offsetX;
    private double offsetY;
    private double offsetHeading;
    private Pose pendingPose;

    /** Supplies the latest HAL state for the tick to the localizer. */
    public void feed(RobotState state) {
        Objects.requireNonNull(state, "state");
        Pose sample = Objects.requireNonNull(state.pinpoint(), "state.pinpoint");
        rawPose = sample;

        if (pendingPose != null) {
            setOffset(pendingPose, sample);
            pendingPose = null;
        }

        Velocity velocity = Velocity.zero();
        if (hasPreviousSample) {
            long dtMs = state.t() - previousTimeMs;
            if (dtMs > 0) {
                double dt = dtMs / 1000.0;
                double rawVx = (sample.x() - previousRawPose.x()) / dt;
                double rawVy = (sample.y() - previousRawPose.y()) / dt;
                double cos = Math.cos(offsetHeading);
                double sin = Math.sin(offsetHeading);
                velocity = new Velocity(
                        rawVx * cos - rawVy * sin,
                        rawVx * sin + rawVy * cos,
                        wrap(sample.heading() - previousRawPose.heading()) / dt);
            }
        }

        motionState = MotionState.ofVelocity(applyOffset(sample), velocity);
        previousRawPose = sample;
        previousTimeMs = state.t();
        hasPreviousSample = true;
    }

    @Override
    public void setPose(Pose pose) {
        Objects.requireNonNull(pose, "pose");
        if (rawPose == null) {
            pendingPose = pose;
            motionState = MotionState.ofVelocity(pose, Velocity.zero());
            return;
        }
        setOffset(pose, rawPose);
        motionState = motionState.withPose(applyOffset(rawPose));
    }

    @Override
    public MotionState state() {
        return motionState;
    }

    /** Intentionally a no-op because this localizer is feed-based. */
    @Override
    public void update() {
        // RobotLoop does not read the same sensor sample twice.
    }

    /** Clears the pose offset; does not reset Pinpoint hardware. */
    @Override
    public void reset() {
        offsetX = 0.0;
        offsetY = 0.0;
        offsetHeading = 0.0;
        pendingPose = null;
        hasPreviousSample = false;
        if (rawPose == null) {
            motionState = MotionState.zero();
        } else {
            motionState = MotionState.ofVelocity(
                    new Pose(rawPose.x(), rawPose.y(), wrap(rawPose.heading())),
                    Velocity.zero());
        }
    }

    private void setOffset(Pose requested, Pose raw) {
        offsetX = requested.x() - raw.x();
        offsetY = requested.y() - raw.y();
        offsetHeading = wrap(requested.heading() - raw.heading());
    }

    private Pose applyOffset(Pose raw) {
        return new Pose(
                raw.x() + offsetX,
                raw.y() + offsetY,
                wrap(raw.heading() + offsetHeading));
    }

    private static double wrap(double angle) {
        double wrapped = angle % (2.0 * Math.PI);
        if (wrapped > Math.PI) {
            wrapped -= 2.0 * Math.PI;
        } else if (wrapped < -Math.PI) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped;
    }
}
