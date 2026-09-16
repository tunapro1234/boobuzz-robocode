package boobuzz.core.logic.cplx1;

import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.ITurret;

import com.pedropathing.math.Pose;

import java.util.Objects;

/** Automatic goal aiming; controllers never issue turret commands directly. */
public final class TurretLogic {

    private final ITurret turret;
    private boolean holdingForShot;
    private boolean targetKnown;
    private double targetX;
    private double targetY;

    public TurretLogic(ITurret turret) {
        this.turret = Objects.requireNonNull(turret, "turret");
    }

    public void update(Pose robotPose) {
        if (holdingForShot) {
            turret.hold();
            return;
        }
        if (robotPose == null) {
            targetKnown = false;
            turret.scan();
            return;
        }
        targetX = RobotConstants.ALLIANCE_BLUE
                ? RobotConstants.GOAL_X : RobotConstants.RED_GOAL_X;
        targetY = RobotConstants.GOAL_Y;
        targetKnown = true;
        turret.aimAt(targetX, targetY);
    }

    public boolean locked() {
        return targetKnown && turret.onTarget();
    }

    public void holdForShot(boolean hold) {
        holdingForShot = hold;
        if (hold) {
            turret.hold();
        }
    }

    public double distanceFrom(Pose pose) {
        if (pose == null || !targetKnown) {
            return 0.0;
        }
        return Math.hypot(targetX - pose.x(), targetY - pose.y());
    }

    public double targetX() {
        return targetX;
    }

    public double targetY() {
        return targetY;
    }
}
