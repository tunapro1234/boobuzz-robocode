package boobuzz.core.controller.auto;

import boobuzz.core.contract.PathRequest;

import com.pedropathing.math.Pose;
import boobuzz.core.hal.RobotConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.UnaryOperator;

/** Small fluent DSL that compiles autonomous routines into immutable auto steps. */
public final class AutoBuilder {

    private static final double SAME_HEADING_EPS_RAD = 0.01;
    private static final double DEFAULT_INTAKE_POWER = 0.8;
    private static final double DEFAULT_SHOOTER_RPM = RobotConstants.SHOOTER_RPM_BASE;

    private final List<AutoStep> steps = new ArrayList<>();
    private Pose currentPose;

    private AutoBuilder(Pose startPose) {
        currentPose = Objects.requireNonNull(startPose, "start pose");
    }

    public static AutoBuilder start(Pose startPose) {
        return new AutoBuilder(startPose);
    }

    /** Treat a pose as a point and discard its heading. */
    public static Pose point(Pose pose) {
        Objects.requireNonNull(pose, "pose");
        return new Pose(pose.x(), pose.y());
    }

    public AutoBuilder goToPose(Pose targetPose) {
        Objects.requireNonNull(targetPose, "target pose");
        double delta = normalizeAngle(targetPose.heading() - currentPose.heading());
        PathRequest.Heading heading = Math.abs(delta) < SAME_HEADING_EPS_RAD
                ? PathRequest.Heading.constant(targetPose.heading())
                : PathRequest.Heading.linear(currentPose.heading(), targetPose.heading());
        addPath(targetPose, heading, List.of());
        return this;
    }

    /** Move to a pose; heading is specified in degrees to match last season's API. */
    public AutoBuilder goTo(double x, double y, double headingDeg) {
        return goToPose(new Pose(x, y, Math.toRadians(headingDeg)));
    }

    /** Follow a straight line; the follower chooses tangential heading. */
    public AutoBuilder lineTo(double x, double y) {
        return lineTo(new Pose(x, y));
    }

    public AutoBuilder lineTo(Pose point) {
        Objects.requireNonNull(point, "line endpoint");
        addPath(point, PathRequest.Heading.tangent(), List.of());
        return this;
    }

    /** Follow a Bezier curve; endpoint and controls are points, not headings. */
    public AutoBuilder curveTo(Pose endPoint, Pose... controlPoints) {
        Objects.requireNonNull(endPoint, "curve endpoint");
        List<Pose> controls = controlPoints == null ? List.of() : List.of(controlPoints);
        addPath(endPoint, PathRequest.Heading.tangent(), controls);
        return this;
    }

    public AutoBuilder curveTo(double x, double y, Pose... controlPoints) {
        return curveTo(new Pose(x, y), controlPoints);
    }

    public AutoBuilder curveToPose(Pose targetPose, Pose... controlPoints) {
        Objects.requireNonNull(targetPose, "curve target pose");
        List<Pose> controls = controlPoints == null ? List.of() : List.of(controlPoints);
        double delta = normalizeAngle(targetPose.heading() - currentPose.heading());
        PathRequest.Heading heading = Math.abs(delta) < SAME_HEADING_EPS_RAD
                ? PathRequest.Heading.constant(targetPose.heading())
                : PathRequest.Heading.linear(currentPose.heading(), targetPose.heading());
        addPath(targetPose, heading, controls);
        return this;
    }

    public AutoBuilder withTangentialHeading() {
        return modifyLastPath(path -> withRequest(path, path.request().withHeading(
                PathRequest.Heading.tangent())));
    }

    public AutoBuilder withTangentialHeadingReverse() {
        return modifyLastPath(path -> withRequest(path, path.request().withHeading(
                PathRequest.Heading.tangentReverse())));
    }

    /** Heading is specified in degrees to match last season's API. */
    public AutoBuilder withConstantHeading(double headingDeg) {
        return modifyLastPath(path -> withRequest(path, path.request().withHeading(
                PathRequest.Heading.constant(Math.toRadians(headingDeg)))));
    }

    /** Both heading endpoints are specified in degrees. */
    public AutoBuilder withLinearHeading(double startHeadingDeg, double endHeadingDeg) {
        return modifyLastPath(path -> withRequest(path, path.request().withHeading(
                PathRequest.Heading.linear(Math.toRadians(startHeadingDeg),
                        Math.toRadians(endHeadingDeg)))));
    }

    /** End heading is in degrees; the start is the path's original start heading. */
    public AutoBuilder withLinearHeadingTo(double endHeadingDeg) {
        return modifyLastPath(path -> withRequest(path, path.request().withHeading(
                PathRequest.Heading.linear(path.startHeadingRad(),
                        Math.toRadians(endHeadingDeg)))));
    }

    public AutoBuilder shoot(int ballCount) {
        steps.add(new AutoStep.Shoot(ballCount));
        return this;
    }

    public AutoBuilder shoot(int ballCount, double rpm) {
        steps.add(new AutoStep.Shoot(ballCount, rpm));
        return this;
    }

    public AutoBuilder waitSeconds(double seconds) {
        steps.add(new AutoStep.Wait(seconds));
        return this;
    }

    public AutoBuilder intake(double seconds) {
        steps.add(new AutoStep.Intake(seconds, DEFAULT_INTAKE_POWER));
        return this;
    }

    public AutoBuilder intake(double seconds, double power) {
        steps.add(new AutoStep.Intake(seconds, power));
        return this;
    }

    /** Turn in place; heading is specified in degrees to match last season's API. */
    public AutoBuilder turnTo(double headingDeg) {
        double headingRad = Math.toRadians(headingDeg);
        steps.add(new AutoStep.Turn(headingRad));
        currentPose = new Pose(currentPose.x(), currentPose.y(), headingRad);
        return this;
    }

    /** Attach intake-on to the most recent path. */
    public AutoBuilder withIntake() {
        return withIntake(DEFAULT_INTAKE_POWER);
    }

    public AutoBuilder withIntake(double power) {
        return modifyLastPath(path -> new AutoStep.Path(path.request(), path.startPose(),
                true, power, path.shooterWarmupRpm()));
    }

    /** Attach shooter warmup to the most recent path; the stub uses the supplied RPM. */
    public AutoBuilder withShooterWarmup() {
        return withShooterWarmup(DEFAULT_SHOOTER_RPM);
    }

    public AutoBuilder withShooterWarmup(double rpm) {
        return modifyLastPath(path -> new AutoStep.Path(path.request(), path.startPose(),
                path.intake(), path.intakePower(), rpm));
    }

    public AutoBuilder withHoldEnd(boolean holdEnd) {
        return modifyLastPath(path -> withRequest(path, path.request().withHoldEnd(holdEnd)));
    }

    public AutoBuilder withPathConstraints(PathRequest.Constraints constraints) {
        Objects.requireNonNull(constraints, "path constraints");
        return modifyLastPath(path -> withRequest(path,
                path.request().withConstraints(constraints)));
    }

    public AutoBuilder withPathConstraints(double maxPower, double maxVelocity) {
        return withPathConstraints(new PathRequest.Constraints(maxPower, maxVelocity));
    }

    public AutoBuilder withVelocityConstraint(double velocityConstraint) {
        return modifyLastPath(path -> withRequest(path,
                path.request().withVelocityConstraint(velocityConstraint)));
    }

    public AutoBuilder withBraking(double strength, double startMultiplier) {
        return modifyLastPath(path -> withRequest(path,
                path.request().withBraking(new PathRequest.Braking(strength, startMultiplier))));
    }

    public AutoSequence build() {
        return new AutoSequence(steps);
    }

    public Pose currentPose() {
        return currentPose;
    }

    private void addPath(Pose endpoint, PathRequest.Heading heading, List<Pose> controls) {
        PathRequest.Segment segment = controls.isEmpty()
                ? PathRequest.line(point(endpoint))
                : new PathRequest.Curve(point(endpoint), controls);
        PathRequest request = new PathRequest(List.of(segment), heading, true, null, null);
        Pose start = currentPose;
        steps.add(new AutoStep.Path(request, start, false,
                DEFAULT_INTAKE_POWER, null));
        currentPose = new Pose(endpoint.x(), endpoint.y(),
                expectedHeading(request, start));
    }

    private static double expectedHeading(PathRequest request, Pose start) {
        PathRequest.Heading heading = request.heading();
        return switch (heading.mode()) {
            case CONSTANT, LINEAR -> heading.end();
            case TANGENT, TANGENT_REVERSE -> {
                List<PathRequest.Segment> segments = request.segments();
                PathRequest.Segment last = segments.get(segments.size() - 1);
                Pose from = segments.size() == 1 ? start
                        : segments.get(segments.size() - 2).end();
                Pose tangentFrom = from;
                if (last instanceof PathRequest.Curve curve && !curve.controlPoints().isEmpty()) {
                    tangentFrom = curve.controlPoints().get(curve.controlPoints().size() - 1);
                }
                double tangent = Math.atan2(last.end().y() - tangentFrom.y(),
                        last.end().x() - tangentFrom.x());
                yield heading.mode() == PathRequest.HeadingMode.TANGENT_REVERSE
                        ? normalizeAngle(tangent + Math.PI) : tangent;
            }
        };
    }

    private AutoBuilder modifyLastPath(UnaryOperator<AutoStep.Path> modifier) {
        for (int i = steps.size() - 1; i >= 0; i--) {
            if (steps.get(i) instanceof AutoStep.Path path) {
                AutoStep.Path updated = modifier.apply(path);
                steps.set(i, updated);
                currentPose = new Pose(currentPose.x(), currentPose.y(),
                        expectedHeading(updated.request(), updated.startPose()));
                return this;
            }
        }
        return this;
    }

    private static AutoStep.Path withRequest(AutoStep.Path path, PathRequest request) {
        return new AutoStep.Path(request, path.startPose(), path.intake(),
                path.intakePower(), path.shooterWarmupRpm());
    }

    private static double normalizeAngle(double radians) {
        while (radians > Math.PI) {
            radians -= 2.0 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2.0 * Math.PI;
        }
        return radians;
    }
}
