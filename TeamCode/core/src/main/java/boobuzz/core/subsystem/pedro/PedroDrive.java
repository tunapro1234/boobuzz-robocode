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
public final class PedroDrive implements boobuzz.core.subsystem.Drive {

    private static final int FL = 0;
    private static final int FR = 1;
    private static final int BL = 2;
    private static final int BR = 3;

    private final String[] motorNames;
    private final HalLocalizer localizer;
    private final HalDrivetrain drivetrain;
    private final Follower follower;
    private final PathRegistry paths;

    private boobuzz.core.contract.Drive activeCommand = boobuzz.core.contract.Drive.HOLD;
    private boobuzz.core.contract.Drive startedCommand;
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
        follower = PedroConstants.createFollower(mechanism, localizer, drivetrain);
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
        if (!(activeCommand instanceof boobuzz.core.contract.Drive.Manual)) {
            follower.stop();
            startedCommand = null;
        }
        activeCommand = new boobuzz.core.contract.Drive.Manual(vx, vy, omega);
    }

    @Override
    public void follow(PathRequest request) {
        Objects.requireNonNull(request, "request");
        activePathRequest = request;
        activeCommand = request.isNamed()
                ? new boobuzz.core.contract.Drive.FollowPath(request.pathId())
                : request.segments().isEmpty()
                        ? new boobuzz.core.contract.Drive.GoTo(request.target(), request.constraints())
                        : boobuzz.core.contract.Drive.HOLD;
    }

    @Override
    public void turnTo(double headingRad) {
        Objects.requireNonNull(pose(), "pose");
        activePathRequest = null;
        startedPathRequest = null;
        follower.stop();
        Pose current = pose();
        activeCommand = new boobuzz.core.contract.Drive.GoTo(
                new Pose(current.x(), current.y(), headingRad),
                boobuzz.core.contract.Drive.Constraints.defaults());
        startedCommand = null;
    }

    @Override
    public void stop() {
        activePathRequest = null;
        startedPathRequest = null;
        activeCommand = boobuzz.core.contract.Drive.HOLD;
        follower.stop();
        startedCommand = activeCommand;
    }

    @Override
    public boolean pathDone() {
        return !follower.isBusy();
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
            follower.update(deltaTimeSeconds);
            writeFollowerAction(out);
            return;
        }

        if (activeCommand instanceof boobuzz.core.contract.Drive.Manual manual) {
            writeManual(manual, out);
            return;
        }

        if (!sameCommand(startedCommand, activeCommand)) {
            start(activeCommand);
            startedCommand = activeCommand;
        }
        follower.update(deltaTimeSeconds);
        writeFollowerAction(out);
    }

    private void writeFollowerAction(RobotAction.Builder out) {
        RobotAction action = drivetrain.lastAction();
        action.motors().forEach(out::motor);
        action.servos().forEach(out::servo);
    }

    private void writeManual(boobuzz.core.contract.Drive.Manual manual,
                             RobotAction.Builder out) {
        double[] powers = HalDrivetrain.normalizedMecanum(
                new DrivePowers(manual.vx(), manual.vy(), manual.omega()));
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
            follower.hold(request.target(), request.holdEnd());
            return;
        }
        follower.holdEnd.set(request.holdEnd());
        follower.follow(withHeadingAndConstraints(buildPath(request), request));
    }

    private void start(boobuzz.core.contract.Drive command) {
        if (command instanceof boobuzz.core.contract.Drive.GoTo goTo) {
            follower.hold(goTo.target());
        } else if (command instanceof boobuzz.core.contract.Drive.FollowPath followPath) {
            paths.start(follower, followPath.pathId());
        } else if (command instanceof boobuzz.core.contract.Drive.Hold) {
            follower.hold(localizer.pose());
        } else {
            follower.stop();
        }
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

    private Path withHeadingAndConstraints(Path path, PathRequest request) {
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
            modifiers.add(foresight.config.maxBrakingPower.at(request.braking().strength()));
            modifiers.add(foresight.config.maxDecelerationScale.at(
                    request.braking().startMultiplier()));
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

    private static boolean sameCommand(boobuzz.core.contract.Drive left,
                                       boobuzz.core.contract.Drive right) {
        if (left == null || right == null || left.getClass() != right.getClass()) {
            return false;
        }
        if (left instanceof boobuzz.core.contract.Drive.FollowPath a
                && right instanceof boobuzz.core.contract.Drive.FollowPath b) {
            return a.pathId().equals(b.pathId());
        }
        if (left instanceof boobuzz.core.contract.Drive.GoTo a
                && right instanceof boobuzz.core.contract.Drive.GoTo b) {
            return samePose(a.target(), b.target()) && a.constraints().equals(b.constraints());
        }
        if (left instanceof boobuzz.core.contract.Drive.Hold) {
            return true;
        }
        return left instanceof boobuzz.core.contract.Drive.Velocity a
                && right instanceof boobuzz.core.contract.Drive.Velocity b && a.equals(b);
    }

    private static boolean samePose(Pose a, Pose b) {
        return Double.doubleToLongBits(a.x()) == Double.doubleToLongBits(b.x())
                && Double.doubleToLongBits(a.y()) == Double.doubleToLongBits(b.y())
                && Double.doubleToLongBits(a.heading()) == Double.doubleToLongBits(b.heading());
    }
}
