package boobuzz.core.pedro;

import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.engine.C1DriveEngine;
import boobuzz.core.engine.RobotEngine;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;
import boobuzz.core.mechanism.Mechanism;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

/**
 * C1 manuel surusu koruyup GoTo/FollowPath/Hold'u Pedro'ya veren engine.
 *
 * <p>{@link Drive.GoTo#constraints()} Faz 2.5'te uygulanmaz; GoTo hedefi Pedro
 * {@code hold(Pose)} ile tutulur. Kisit destegi olcumden sonra eklenmelidir.
 */
public final class PedroDriveEngine implements RobotEngine {

    private final C1DriveEngine manualEngine;
    private final HalLocalizer localizer;
    private final HalDrivetrain drivetrain;
    private final Follower follower;
    private final PathRegistry paths;

    private Drive activeDrive;
    private long previousStateTimeMs;
    private boolean hasPreviousState;
    private double deltaTimeSeconds;

    public PedroDriveEngine(Mechanism mechanism) {
        this(mechanism, new PathRegistry());
    }

    public PedroDriveEngine(Mechanism mechanism, PathRegistry paths) {
        this.manualEngine = new C1DriveEngine(mechanism);
        this.localizer = new HalLocalizer();
        this.drivetrain = new HalDrivetrain(mechanism);
        this.follower = PedroConstants.createFollower(localizer, drivetrain);
        this.paths = paths;
    }

    @Override
    public String name() {
        return "PedroDrive";
    }

    @Override
    public Feedback sense(long now, RobotState state) {
        localizer.feed(state);
        deltaTimeSeconds = 0.0;
        if (hasPreviousState && state.t() > previousStateTimeMs) {
            deltaTimeSeconds = (state.t() - previousStateTimeMs) / 1000.0;
        }
        previousStateTimeMs = state.t();
        hasPreviousState = true;

        Feedback base = manualEngine.sense(now, state);
        WorldSnapshot world = new WorldSnapshot(
                state.t(), localizer.pose(), state.yaw(), state.voltage());
        return new Feedback(world, base.statuses(), now);
    }

    @Override
    public RobotAction act(Intent intent) {
        RobotAction manualAction = manualEngine.act(intent);
        Drive drive = intent.drive();
        if (drive instanceof Drive.Manual) {
            follower.stop();
            activeDrive = null;
            return manualAction;
        }

        if (!sameCommand(activeDrive, drive)) {
            start(drive);
            activeDrive = drive;
        }

        // Sistem duvar saatiyle degil HAL'in deterministik tick suresiyle ilerler.
        follower.update(deltaTimeSeconds);
        return drivetrain.lastAction();
    }

    private void start(Drive drive) {
        if (drive instanceof Drive.GoTo goTo) {
            follower.hold(goTo.target());
        } else if (drive instanceof Drive.FollowPath followPath) {
            paths.start(follower, followPath.pathId());
        } else if (drive instanceof Drive.Hold) {
            follower.hold(localizer.pose());
        } else {
            // Drive.Velocity Faz 7'nin kapali cevrim isidir; burada sessiz hareket etmez.
            follower.stop();
        }
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
        if (left instanceof Drive.Velocity a && right instanceof Drive.Velocity b) {
            return a.equals(b);
        }
        return false;
    }

    private static boolean samePose(Pose a, Pose b) {
        return Double.doubleToLongBits(a.x()) == Double.doubleToLongBits(b.x())
                && Double.doubleToLongBits(a.y()) == Double.doubleToLongBits(b.y())
                && Double.doubleToLongBits(a.heading()) == Double.doubleToLongBits(b.heading());
    }
}
