package boobuzz.core.subsystem.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.Map;

/** Small registry of named Pedro paths defined in code. */
public final class PathRegistry {

    private static final String TEST_LINE = "test-line";
    private static final String TEST_TURN = "test-turn";

    private final Map<String, Path> paths;
    private final Map<String, Pose> holdTargets;

    public PathRegistry() {
        Pose lineStart = new Pose(72, 72, 0);
        Pose lineEnd = new Pose(120, 72, 0);
        Map<String, Path> pathMap = new java.util.LinkedHashMap<>();
        pathMap.put(TEST_LINE, Paths.line(lineStart, lineEnd).constant(0));
        paths = pathMap;

        // Pedro 3.0 rejects zero-length lines. A turn at the same (x,y) is
        // represented by the library's direct hold(Pose) method.
        Map<String, Pose> targetMap = new java.util.LinkedHashMap<>();
        targetMap.put(TEST_TURN, new Pose(120, 72, Math.PI / 2.0));
        holdTargets = targetMap;
    }

    /** Starts a registered command on the follower; unknown IDs are not swallowed. */
    public void start(Follower follower, String id) {
        Path path = paths.get(id);
        if (path != null) {
            follower.follow(path);
            return;
        }
        Pose holdTarget = holdTargets.get(id);
        if (holdTarget != null) {
            startHold(follower, holdTarget, false);
            return;
        }
        throw new IllegalArgumentException("unknown Pedro path: " + id);
    }

    /**
     * Starts an in-place hold that reports busy until it settles or times out.
     *
     * <p>Pedro core 3.0.0 (javap): {@code Follower.follow} calls
     * {@code algorithm.reset()}, which sets Foresight {@code busy = true}, but
     * {@code Follower.hold} only clears follower state and never touches the
     * algorithm. On a fresh follower (busy starts false) or after a finished path
     * (busy already cleared), a bare hold would report done on its first tick,
     * before any turning. Resetting first gives the hold the same end conditions
     * as a path end: heading, translational and velocity constraints, or the hold
     * timeout. The archive ran a turn as a busy {@code followPath}, not a hold.
     */
    static void startHold(Follower follower, Pose target, boolean useHoldScaling) {
        follower.algorithm().reset();
        follower.hold(target, useHoldScaling);
    }
}
