package boobuzz.core.contract;

import com.pedropathing.math.Pose;

/** Dunyanin o anki hali. */
public record WorldSnapshot(long t, Pose pose, double yaw, double voltage) {
}
