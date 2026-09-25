package boobuzz.core.subsystem.pedro;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.Mechanism;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.config.Modifier;
import com.pedropathing.api.Paths;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** Drive subsystem backed by the Pedro follower and the HAL motor seam. */
public final class PedroDrive implements boobuzz.core.subsystem.IDrive {

    private static final int FL = 0;
    private static final int FR = 1;
    private static final int BL = 2;
    private static final int BR = 3;

    private final String[] motorNames;
    private final HalLocalizer localizer;
    private final HalDrivetrain drivetrain;
    private final Follower follower;
    private final PathRegistry paths;

    private boolean manualActive;
    private double manualVx;
    private double manualVy;
    private double manualOmega;
    private PathRequest activePathRequest;
    private PathRequest startedPathRequest;
    private long previousStateTimeMs;
    private boolean hasPreviousState;
    private double deltaTimeSeconds;

    public PedroDrive(Mechanism mechanism) {
        this(mechanism, new PathRegistry());
    }

    public PedroDrive(Mechanism mechanism, PathRegistry paths) {
        Objects.requireNonNull(mechanism, "mechanism");
        this.motorNames = wheelNames(mechanism);
        this.paths = Objects.requireNonNull(paths, "paths");
        localizer = new HalLocalizer();
        drivetrain = new HalDrivetrain(motorNames);
        // Pedro's hold timeout reads this HAL tick time, never System.nanoTime.
        follower = PedroConstants.createFollower(
                mechanism, localizer, drivetrain, () -> previousStateTimeMs);
    }

    @Override
    public void observe(RobotState state) {
        localizer.feed(state);
        deltaTimeSeconds = 0.0;
        if (hasPreviousState && state.t() > previousStateTimeMs) {
            deltaTimeSeconds = (state.t() - previousStateTimeMs) / 1000.0;
        }
        previousStateTimeMs = state.t();
        hasPreviousState = true;
    }

    @Override
    public void manual(double vx, double vy, double omega) {
        activePathRequest = null;
        startedPathRequest = null;
        if (!manualActive) {
            follower.stop();
        }
        manualActive = true;
        manualVx = vx;
        manualVy = vy;
        manualOmega = omega;
    }

    @Override
    public void follow(PathRequest request) {
        Objects.requireNonNull(request, "request");
        manualActive = false;
        activePathRequest = request;
        // Every follow() is a new request and restarts from the current pose, even
        // when it equals the request whose end hold is still running.
        startedPathRequest = null;
    }

    @Override
    public void turnTo(double headingRad) {
        Objects.requireNonNull(pose(), "pose");
        manualActive = false;
        activePathRequest = null;
        startedPathRequest = null;
        follower.stop();
        Pose current = pose();
        activePathRequest = PathRequest.goTo(
                new Pose(current.x(), current.y(), headingRad),
                PathRequest.Constraints.defaults());
    }

    @Override
    public void stop() {
        activePathRequest = null;
        startedPathRequest = null;
        manualActive = false;
        follower.stop();
    }

    @Override
    public void resetPose(Pose pose) {
        Objects.requireNonNull(pose, "pose");
        activePathRequest = null;
        startedPathRequest = null;
        manualActive = false;
        follower.stop();
        localizer.setPose(pose);
    }

    @Override
    public boolean pathDone() {
        // Pedro core 3.0.0 (javap): when a path ends with holdEnd=false the Follower
        // calls stop() and goes IDLE, but Foresight's last calculatePath() already
        // called reset(), which leaves busy=true. Without the idle() check such a
        // path would never report done; the archive's Pedro 2.0.4 breakFollowing()
        // cleared isBusy at the same point.
        return !follower.isBusy() || follower.idle();
    }

    @Override
    public Pose pose() {
        return localizer.pose();
    }

    @Override
    public void update(RobotAction.Builder out) {
        if (activePathRequest != null) {
            if (!Objects.equals(startedPathRequest, activePathRequest)) {
                start(activePathRequest);
                startedPathRequest = activePathRequest;
            }
            // The first HAL sample (and repeated timestamps) has no elapsed time.
            // Pedro's follower divides by dt while generating wheel powers, so
            // updating it at zero would create NaN before HalDrivetrain can help.
            if (deltaTimeSeconds > 0.0) {
                follower.update(deltaTimeSeconds);
            } else {
                drivetrain.stop();
            }
            writeFollowerAction(out);
            return;
        }

        if (manualActive) {
            writeManual(manualVx, manualVy, manualOmega, out);
            return;
        }
    }

    private void writeFollowerAction(RobotAction.Builder out) {
        RobotAction action = drivetrain.lastAction();
        action.motors().forEach(out::motor);
        action.servos().forEach(out::servo);
    }

    private void writeManual(double vx, double vy, double omega,
                             RobotAction.Builder out) {
        double[] powers = HalDrivetrain.normalizedMecanum(
                new DrivePowers(vx, vy, omega));
        for (int i = 0; i < powers.length; i++) {
            out.motor(motorNames[i], powers[i]);
        }
    }

    private void start(PathRequest request) {
        if (request.isNamed()) {
            paths.start(follower, request.pathId());
            return;
        }
        if (request.segments().isEmpty()) {
            Pose target = request.target();
            Pose start = localizer.pose();
            if (Math.abs(target.x() - start.x()) < 1e-9
                    && Math.abs(target.y() - start.y()) < 1e-9) {
                // In-place turn: no hold scaling, like Pedro's own path-end hold(Pose).
                // holdEnd is not a gain flag and has no meaning for a pure hold.
                PathRegistry.startHold(follower, target, false);
            } else {
                follower.holdEnd.set(request.holdEnd());
                follower.follow(withHeadingAndConstraints(
                        Paths.line(start, target), request));
            }
            return;
        }
        follower.holdEnd.set(request.holdEnd());
        follower.follow(withHeadingAndConstraints(buildPath(request), request));
    }

    private Path buildPath(PathRequest request) {
        Pose start = localizer.pose();
        List<Path> pieces = new ArrayList<>(request.segments().size());
        Pose previous = start;
        for (PathRequest.Segment segment : request.segments()) {
            if (segment instanceof PathRequest.Line line) {
                pieces.add(Paths.line(previous, line.end()));
            } else if (segment instanceof PathRequest.Curve curve) {
                List<Pose> points = new ArrayList<>(curve.controlPoints().size() + 2);
                points.add(previous);
                points.addAll(curve.controlPoints());
                points.add(curve.end());
                pieces.add(points.size() == 2
                        ? Paths.line(previous, curve.end())
                        : Paths.curve(points.toArray(Pose[]::new)));
            }
            previous = segment.end();
        }
        return Paths.path(pieces.toArray(Path[]::new));
    }

    /** Package-private for tests: the follower whose config path modifiers target. */
    Follower follower() {
        return follower;
    }

    Path withHeadingAndConstraints(Path path, PathRequest request) {
        PathRequest.Heading heading = request.heading();
        path = switch (heading.mode()) {
            case TANGENT -> path.tangent();
            case TANGENT_REVERSE -> path.reverseTangent();
            case CONSTANT -> path.constant(heading.start());
            // Pedro 3.0's linear interpolator takes (end, start), despite the
            // method name; reverse the contract values to preserve start -> end.
            case LINEAR -> path.linear(heading.end(), heading.start());
        };
        if (!(follower.algorithm() instanceof Foresight foresight)) {
            return path;
        }
        List<Modifier> modifiers = new ArrayList<>(3);
        if (request.velocityConstraint() != null) {
            modifiers.add(foresight.config.velocityConstraint.at(request.velocityConstraint()));
        }
        if (request.braking() != null) {
            // Archive Pedro 2.0.4 (javap ErrorCalculator): brakingStrength scales the
            // zero-power deceleration used to plan the stop; Pedro 3.0 expresses that
            // as maxDecelerationScale, the same mapping as simple-code's
            // AutoPathPlanner.maxDecelerationScale. maxBrakingPower (a cap on reverse
            // power, default 0.2) stays at the Pedro default.
            // startMultiplier is deliberately unused: in 2.0.4 it multiplied the
            // stopping distance (larger = brake earlier), and Pedro 3.0 has no
            // equivalent; mapping it to maxDecelerationScale reversed its direction.
            modifiers.add(foresight.config.maxDecelerationScale.at(
                    request.braking().strength()));
        }
        return modifiers.isEmpty() ? path : path.with(modifiers.toArray(Modifier[]::new));
    }

    static String[] wheelNames(Mechanism mechanism) {
        List<String> wheels = mechanism.wheelMotorNames();
        if (wheels.size() != 4) {
            throw new Mechanism.MechanismException(
                    "PedroDrive requires four 'drives: wheel' motors, found " + wheels.size()
                            + ": " + wheels);
        }
        String[] names = new String[4];
        for (String name : wheels) {
            Mechanism.Motor motor = mechanism.motor(name);
            int index = motor.forward() >= 0
                    ? (motor.left() >= 0 ? FL : FR)
                    : (motor.left() >= 0 ? BL : BR);
            if (names[index] != null) {
                throw new Mechanism.MechanismException(
                        "Two motors map to the same corner: " + names[index] + " and " + name);
            }
            names[index] = name;
        }
        for (String name : names) {
            if (name == null) {
                throw new Mechanism.MechanismException(
                        "Wheel positions do not map to four distinct corners: " + wheels);
            }
        }
        return names;
    }

}
