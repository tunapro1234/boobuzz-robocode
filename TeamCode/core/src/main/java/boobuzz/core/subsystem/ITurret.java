package boobuzz.core.subsystem;

import com.pedropathing.math.Pose;

/** Narrow turret mechanism API; aiming is owned by the complex logic layer. */
public interface ITurret extends ISubsystem {

    /** Outcome of the latest aim request; out-of-range targets are rejected, never clamped. */
    enum AimResult {
        ACCEPTED,
        OUT_OF_RANGE,
        INVALID_INPUT,
        NOT_INITIALIZED
    }

    /**
     * The localized robot pose used for field aiming (drive pose, including any
     * RESET_POSE offset); engines supply it every tick after observe. Null or non-finite
     * makes a field aim INVALID_INPUT.
     */
    void setRobotPose(Pose pose);

    /** Track a field point; reachability is re-evaluated every tick, see {@link #aimStatus()}. */
    void aimAt(double fieldX, double fieldY);

    /** Hold a fixed angle relative to the robot heading (radians, CCW positive). */
    AimResult aimRelative(double angleRad);

    /** Latest reachability of the active aim request. */
    AimResult aimStatus();

    void scan();

    /** Cancel aim/scan ownership and hold the current angle closed-loop. */
    void hold();

    /** Zero both outputs; nothing restarts until a new explicit aim or hold. */
    void disable();

    boolean onTarget();

    double angleRad();
}
