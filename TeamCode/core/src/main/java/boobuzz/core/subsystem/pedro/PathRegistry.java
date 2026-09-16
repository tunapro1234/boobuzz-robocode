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
        paths = Map.of(TEST_LINE, Paths.line(lineStart, lineEnd).constant(0));

        // Pedro 3.0 rejects zero-length lines. A turn at the same (x,y) is
        // represented by the library's direct hold(Pose) method.
        holdTargets = Map.of(TEST_TURN, new Pose(120, 72, Math.PI / 2.0));
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
            follower.hold(holdTarget);
            return;
        }
        throw new IllegalArgumentException("unknown Pedro path: " + id);
    }
}
