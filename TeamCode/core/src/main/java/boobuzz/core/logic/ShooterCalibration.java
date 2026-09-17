package boobuzz.core.logic;

import boobuzz.core.hal.RobotConstants;

import com.pedropathing.math.Pose;

/** Shared count-only shooter calibration used by both engine implementations. */
public final class ShooterCalibration {

    private ShooterCalibration() {}

    public static double calibratedRpm(Pose pose) {
        double distance = pose == null ? 0.0 : Math.hypot(
                (RobotConstants.ALLIANCE_BLUE ? RobotConstants.GOAL_X : RobotConstants.RED_GOAL_X)
                        - pose.x(),
                RobotConstants.GOAL_Y - pose.y());
        return RobotConstants.SHOOTER_RPM_BASE
                + RobotConstants.SHOOTER_RPM_PER_IN * distance;
    }
}
