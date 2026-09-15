package boobuzz.core.logic.cplx_engine_1;

import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Intent;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;
import boobuzz.core.logic.Subsystem;
import boobuzz.core.hal.Mechanism;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.List;

/** Manuel mecanum ve istege bagli Pedro takibini tek L2 drive biriminde toplar. */
public final class DriveSubsystem implements Subsystem {

    private static final int FL = 0;
    private static final int FR = 1;
    private static final int BL = 2;
    private static final int BR = 3;

    private final String[] motorNames;
    private final HalLocalizer localizer;
    private final HalDrivetrain drivetrain;
    private final Follower follower;
    private final PathRegistry paths;

    private List<RequestStatus> pendingStatuses = List.of();
    private Drive activeDrive;
    private long previousStateTimeMs;
    private boolean hasPreviousState;
    private double deltaTimeSeconds;

    private DriveSubsystem(Mechanism mechanism, PathRegistry paths) {
        this.motorNames = wheelNames(mechanism);
        this.paths = paths;
        localizer = new HalLocalizer();
        drivetrain = new HalDrivetrain(motorNames);
        follower = PedroConstants.createFollower(mechanism, localizer, drivetrain);
    }

    public static DriveSubsystem pedro(Mechanism mechanism, PathRegistry paths) {
        return new DriveSubsystem(mechanism, paths);
    }

    @Override
    public String name() {
        return "drive";
    }

    @Override
    public void observe(long now, RobotState state) {
        localizer.feed(state);
        deltaTimeSeconds = 0.0;
        if (hasPreviousState && state.t() > previousStateTimeMs) {
            deltaTimeSeconds = (state.t() - previousStateTimeMs) / 1000.0;
        }
        previousStateTimeMs = state.t();
        hasPreviousState = true;
    }

    @Override
    public void update(Intent intent, RobotAction.Builder out) {
        rejectUnsupportedRequests(intent);
        Drive drive = intent.drive();
        if (drive instanceof Drive.Manual) {
            if (!(activeDrive instanceof Drive.Manual)) {
                follower.stop();
            }
            activeDrive = drive;
            writeManual(drive, out);
            return;
        }

        if (!sameCommand(activeDrive, drive)) {
            start(drive);
            activeDrive = drive;
        }
        follower.update(deltaTimeSeconds);
        write(drivetrain.lastAction(), out);
    }

    public List<RequestStatus> drainStatuses() {
        List<RequestStatus> statuses = pendingStatuses;
        pendingStatuses = List.of();
        return statuses;
    }

    public Pose pose() {
        return localizer.pose();
    }

    Drive activeCommand() { return activeDrive; }

    Follower.Mode followerMode() { return follower.mode(); }

    double deltaTimeSeconds() { return deltaTimeSeconds; }

    private void rejectUnsupportedRequests(Intent intent) {
        if (intent.newRequests().isEmpty()) {
            return;
        }
        List<RequestStatus> statuses = new ArrayList<>(intent.newRequests().size());
        for (Request request : intent.newRequests()) {
            statuses.add(RequestStatus.rejected(
                    request.id(), "cplx_engine_1 bu istek icin subsystem tasimiyor"));
        }
        pendingStatuses = List.copyOf(statuses);
    }

    private void writeManual(Drive drive, RobotAction.Builder out) {
        double vx = 0.0;
        double vy = 0.0;
        double omega = 0.0;
        if (drive instanceof Drive.Manual manual) {
            vx = manual.vx();
            vy = manual.vy();
            omega = manual.omega();
        }

        double[] powers = {
                vx - vy - omega,
                vx + vy + omega,
                vx + vy - omega,
                vx - vy + omega};
        double peak = 1.0;
        for (double power : powers) {
            peak = Math.max(peak, Math.abs(power));
        }
        for (int i = 0; i < powers.length; i++) {
            out.motor(motorNames[i], powers[i] / peak);
        }
    }

    private void start(Drive drive) {
        if (drive instanceof Drive.GoTo goTo) {
            follower.hold(goTo.target());
        } else if (drive instanceof Drive.FollowPath followPath) {
            paths.start(follower, followPath.pathId());
        } else if (drive instanceof Drive.Hold) {
            follower.hold(localizer.pose());
        } else {
            follower.stop();
        }
    }

    private static void write(RobotAction action, RobotAction.Builder out) {
        action.motors().forEach(out::motor);
        action.servos().forEach(out::servo);
    }

    static String[] wheelNames(Mechanism mechanism) {
        List<String> wheels = mechanism.wheelMotorNames();
        if (wheels.size() != 4) {
            throw new Mechanism.MechanismException(
                    "DriveSubsystem dort adet 'drives: wheel' motoru bekler, " + wheels.size()
                            + " buldu: " + wheels);
        }
        String[] names = new String[4];
        for (String name : wheels) {
            Mechanism.Motor motor = mechanism.motor(name);
            int index = motor.forward() >= 0
                    ? (motor.left() >= 0 ? FL : FR)
                    : (motor.left() >= 0 ? BL : BR);
            if (names[index] != null) {
                throw new Mechanism.MechanismException(
                        "Iki motor ayni koseye dusuyor: " + names[index] + " ve " + name);
            }
            names[index] = name;
        }
        for (String name : names) {
            if (name == null) {
                throw new Mechanism.MechanismException(
                        "Tekerlek konumlari dort ayri koseye dusmuyor: " + wheels);
            }
        }
        return names;
    }

    private static boolean sameCommand(Drive left, Drive right) {
        if (left == null || left.getClass() != right.getClass()) {
            return false;
        }
        if (left instanceof Drive.FollowPath a && right instanceof Drive.FollowPath b) {
            return a.pathId().equals(b.pathId());
        }
        if (left instanceof Drive.GoTo a && right instanceof Drive.GoTo b) {
            return samePose(a.target(), b.target()) && a.constraints().equals(b.constraints());
        }
        if (left instanceof Drive.Hold) {
            return true;
        }
        return left instanceof Drive.Velocity a && right instanceof Drive.Velocity b
                && a.equals(b);
    }

    private static boolean samePose(Pose a, Pose b) {
        return Double.doubleToLongBits(a.x()) == Double.doubleToLongBits(b.x())
                && Double.doubleToLongBits(a.y()) == Double.doubleToLongBits(b.y())
                && Double.doubleToLongBits(a.heading()) == Double.doubleToLongBits(b.heading());
    }
}
