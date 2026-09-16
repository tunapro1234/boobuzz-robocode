package boobuzz.core.contract;

import com.pedropathing.math.Pose;

import java.util.List;
import java.util.Objects;

/** Immutable path description shared by the auto controller and drive subsystem. */
public record PathRequest(
        String pathId,
        Pose target,
        Drive.Constraints constraints,
        List<Segment> segments,
        Heading heading,
        boolean holdEnd,
        Double velocityConstraint,
        Braking braking) {

    public PathRequest {
        if (pathId != null && pathId.isBlank()) {
            throw new IllegalArgumentException("path request ID must not be blank");
        }
        constraints = constraints == null ? Drive.Constraints.defaults() : constraints;
        segments = segments == null ? List.of() : List.copyOf(segments);
        heading = heading == null ? Heading.tangent() : heading;
        for (Segment segment : segments) {
            Objects.requireNonNull(segment, "path segment");
        }
        if (target == null && !segments.isEmpty()) {
            target = segments.get(segments.size() - 1).end();
        }
        if (pathId == null && target == null && segments.isEmpty()) {
            throw new IllegalArgumentException("path request needs a path ID or target pose");
        }
        if (velocityConstraint != null
                && (!Double.isFinite(velocityConstraint) || velocityConstraint <= 0.0)) {
            throw new IllegalArgumentException("velocity constraint must be positive and finite");
        }
    }

    /** Legacy direct-target constructor used by {@link Drive.GoTo}. */
    public PathRequest(Pose target, Drive.Constraints constraints) {
        this(null, Objects.requireNonNull(target, "target"), constraints,
                List.of(), Heading.constant(target.heading()), true, null, null);
    }

    /** Legacy named-path constructor. */
    public PathRequest(String pathId) {
        this(pathId, null, Drive.Constraints.defaults(), List.of(), Heading.tangent(),
                true, null, null);
    }

    /** Compatibility constructor retained for older callers. */
    public PathRequest(String pathId, Pose target, Drive.Constraints constraints) {
        this(pathId, target, constraints, List.of(), Heading.tangent(), true, null, null);
    }

    /** Full path constructor for the auto request contract. */
    public PathRequest(List<Segment> segments, Heading heading, boolean holdEnd,
                       Double velocityConstraint, Braking braking) {
        this(null, null, Drive.Constraints.defaults(), segments, heading, holdEnd,
                velocityConstraint, braking);
    }

    public static PathRequest goTo(Pose target, Drive.Constraints constraints) {
        return new PathRequest(target, constraints);
    }

    public static PathRequest named(String pathId) {
        return new PathRequest(pathId);
    }

    public static PathRequest path(List<Segment> segments, Heading heading, boolean holdEnd,
                                   Double velocityConstraint, Braking braking) {
        return new PathRequest(segments, heading, holdEnd, velocityConstraint, braking);
    }

    public static PathRequest path(List<Segment> segments) {
        return path(segments, Heading.tangent(), true, null, null);
    }

    public static Line line(Pose end) {
        return new Line(end);
    }

    public static Curve curve(Pose end, Pose... controlPoints) {
        return new Curve(end, controlPoints == null ? List.of() : List.of(controlPoints));
    }

    public boolean isNamed() {
        return pathId != null;
    }

    public enum HeadingMode {
        TANGENT,
        TANGENT_REVERSE,
        CONSTANT,
        LINEAR
    }

    /** Heading interpolation plus the values used by CONSTANT and LINEAR. */
    public record Heading(HeadingMode mode, double start, double end) {
        public Heading {
            Objects.requireNonNull(mode, "heading mode");
            if ((mode == HeadingMode.CONSTANT || mode == HeadingMode.LINEAR)
                    && (!Double.isFinite(start) || !Double.isFinite(end))) {
                throw new IllegalArgumentException("heading values must be finite");
            }
        }

        public static Heading tangent() {
            return new Heading(HeadingMode.TANGENT, 0.0, 0.0);
        }

        public static Heading tangentReverse() {
            return new Heading(HeadingMode.TANGENT_REVERSE, 0.0, 0.0);
        }

        public static Heading constant(double heading) {
            return new Heading(HeadingMode.CONSTANT, heading, heading);
        }

        public static Heading linear(double start, double end) {
            return new Heading(HeadingMode.LINEAR, start, end);
        }
    }

    /** Braking strength and the distance multiplier at which braking starts. */
    public record Braking(double strength, double startMultiplier) {
        public Braking {
            if (!Double.isFinite(strength) || !Double.isFinite(startMultiplier)) {
                throw new IllegalArgumentException("braking values must be finite");
            }
        }
    }

    /** One ordered geometric segment; endpoints are in field inches. */
    public sealed interface Segment permits Line, Curve {
        Pose end();
    }

    /** Straight segment ending at a point (the pose heading is ignored). */
    public record Line(Pose end) implements Segment {
        public Line {
            Objects.requireNonNull(end, "line end");
        }
    }

    /** Bezier segment ending at a point with zero or more control points. */
    public record Curve(Pose end, List<Pose> controlPoints) implements Segment {
        public Curve {
            Objects.requireNonNull(end, "curve end");
            controlPoints = controlPoints == null ? List.of() : List.copyOf(controlPoints);
            for (Pose point : controlPoints) {
                Objects.requireNonNull(point, "curve control point");
            }
        }

        public Curve(Pose end, Pose... controlPoints) {
            this(end, controlPoints == null ? List.of() : List.of(controlPoints));
        }
    }
}
