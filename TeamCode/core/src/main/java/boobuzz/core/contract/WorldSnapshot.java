package boobuzz.core.contract;

import com.pedropathing.math.Pose;

/** Current state of the world. */
public record WorldSnapshot(long t, Pose pose, double yaw, double voltage) {
}
