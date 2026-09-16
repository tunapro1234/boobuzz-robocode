package boobuzz.core.contract;

import com.pedropathing.math.Pose;

/**
 * Drive intent. Architecture documentation, §6.
 *
 * <p>Sign convention matches Pedro: {@code vx} FORWARD, {@code vy} LEFT, {@code omega} CCW.
 * {@code TeleopController} first converts raw sticks {@code (-ly, -lx, -rx)} to this convention.
 */
public sealed interface Drive {

    /** Robot frame, raw power from -1 to 1. */
    record Manual(double vx, double vy, double omega) implements Drive {}

    /**
     * Field frame, in/s. RL uses this (architecture documentation, §6).
     * Must be CLOSED LOOP; an open-loop power mapping silently becomes
     * "power" and drifts by 20%+ with battery voltage.
     */
    record Velocity(double vx, double vy, double omega) implements Drive {}

    /** Go to a point with the Pedro follower. */
    record GoTo(Pose target, Constraints constraints) implements Drive {}

    /** Follow a predefined path. */
    record FollowPath(String pathId) implements Drive {}

    /** Hold the current position. */
    record Hold() implements Drive {}

    /** Motion constraints. */
    record Constraints(double maxPower, double maxVelocity) {
        public static Constraints defaults() {
            return new Constraints(1.0, Double.MAX_VALUE);
        }
    }

    Drive HOLD = new Hold();
}
