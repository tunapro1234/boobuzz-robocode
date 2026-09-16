package boobuzz.core.contract;

import com.pedropathing.math.Pose;

import java.util.Objects;

/** A drive request that can identify a registered path or a direct target pose. */
public record PathRequest(String pathId, Pose target, Drive.Constraints constraints) {

    public PathRequest {
        if (pathId == null && target == null) {
            throw new IllegalArgumentException("path request needs a path ID or target pose");
        }
        if (pathId != null && pathId.isBlank()) {
            throw new IllegalArgumentException("path request ID must not be blank");
        }
        if (target != null) {
            Objects.requireNonNull(target, "target");
        }
        constraints = constraints == null ? Drive.Constraints.defaults() : constraints;
    }

    public PathRequest(Pose target, Drive.Constraints constraints) {
        this(null, target, constraints);
    }

    public PathRequest(String pathId) {
        this(pathId, null, Drive.Constraints.defaults());
    }

    public static PathRequest goTo(Pose target, Drive.Constraints constraints) {
        return new PathRequest(target, constraints);
    }

    public static PathRequest named(String pathId) {
        return new PathRequest(pathId);
    }

    public boolean isNamed() {
        return pathId != null;
    }
}
