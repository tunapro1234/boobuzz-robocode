package boobuzz.core.logic.cplx_engine_1;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.Map;

/** Kod icinde adlandirilmis, kucuk Pedro yol kaydi. */
public final class PathRegistry {

    private static final String TEST_LINE = "test-line";
    private static final String TEST_TURN = "test-turn";

    private final Map<String, Path> paths;
    private final Map<String, Pose> holdTargets;

    public PathRegistry() {
        Pose lineStart = new Pose(72, 72, 0);
        Pose lineEnd = new Pose(120, 72, 0);
        paths = Map.of(TEST_LINE, Paths.line(lineStart, lineEnd).constant(0));

        // Pedro 3.0 Line sifir uzunlugu reddeder. Ayni (x,y)'de donus, kutuphanenin
        // dogrudan sundugu hold(Pose) ile ifade edilir.
        holdTargets = Map.of(TEST_TURN, new Pose(120, 72, Math.PI / 2.0));
    }

    /** Kayitli komutu follower'da baslatir; bilinmeyen kimlik sessizce yutulmaz. */
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
        throw new IllegalArgumentException("bilinmeyen Pedro yolu: " + id);
    }
}
