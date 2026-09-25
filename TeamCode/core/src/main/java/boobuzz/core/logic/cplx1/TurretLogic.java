package boobuzz.core.logic.cplx1;

import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.ITurret;

import com.pedropathing.math.Pose;

import java.util.Objects;

/** Automatic goal aiming; controllers never issue turret commands directly. */
public final class TurretLogic {

    /** What the turret aims at; set explicitly, never overwritten by update(). */
    public enum TargetMode { ALLIANCE_GOAL, FIELD_POINT, FIXED_RELATIVE }

    private final ITurret turret;
    private boolean holdingForShot;
    private boolean enabled = true;
    private TargetMode mode = TargetMode.ALLIANCE_GOAL;
    private double fieldX;
    private double fieldY;
    private double relativeRad;
    private boolean targetKnown;
    private double targetX;
    private double targetY;

    public TurretLogic(ITurret turret) {
        this.turret = Objects.requireNonNull(turret, "turret");
    }

    /** Default: the alliance goal from RobotConstants. */
    public void useAllianceGoal() {
        mode = TargetMode.ALLIANCE_GOAL;
        enabled = true;
    }

    /** Zero the turret; update() never re-arms it until a fresh target or enable(). */
    public void disable() {
        enabled = false;
        holdingForShot = false;
        targetKnown = false;
        turret.disable();
    }

    /** Resume aiming at the current target selection after disable(). */
    public void enable() {
        enabled = true;
    }

    public boolean enabled() {
        return enabled;
    }

    /** False while the turret mechanism is still calibrating (no aim can be accepted). */
    public boolean startupDone() {
        return turret.aimStatus() != ITurret.AimResult.NOT_INITIALIZED;
    }

    /** Track a field point; the same setter a future shot solver uses. */
    public void setFieldTarget(double x, double y) {
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            throw new IllegalArgumentException("field target must be finite");
        }
        mode = TargetMode.FIELD_POINT;
        fieldX = x;
        fieldY = y;
        enabled = true;
    }

    /** Fixed preset relative to the robot heading (radians, CCW positive). */
    public void setRelativeTarget(double angleRad) {
        if (!Double.isFinite(angleRad)) {
            throw new IllegalArgumentException("relative target must be finite");
        }
        mode = TargetMode.FIXED_RELATIVE;
        relativeRad = angleRad;
        enabled = true;
    }

    public double relativeTargetRad() {
        return relativeRad;
    }

    public TargetMode targetMode() {
        return mode;
    }

    public void update(Pose robotPose) {
        if (!enabled) {
            targetKnown = false;
            return;
        }
        if (holdingForShot) {
            turret.hold();
            return;
        }
        if (mode == TargetMode.FIXED_RELATIVE) {
            targetKnown = false;
            turret.aimRelative(relativeRad);
            return;
        }
        if (robotPose == null) {
            targetKnown = false;
            turret.scan();
            return;
        }
        if (mode == TargetMode.FIELD_POINT) {
            targetX = fieldX;
            targetY = fieldY;
        } else {
            targetX = RobotConstants.ALLIANCE_BLUE
                    ? RobotConstants.GOAL_X : RobotConstants.RED_GOAL_X;
            targetY = RobotConstants.GOAL_Y;
        }
        targetKnown = true;
        turret.aimAt(targetX, targetY);
    }

    /** On target AND the active aim is reachable (never a clamped "success"). */
    public boolean locked() {
        boolean aimValid = mode == TargetMode.FIXED_RELATIVE || targetKnown;
        return enabled && aimValid && turret.aimStatus() == ITurret.AimResult.ACCEPTED
                && turret.onTarget();
    }

    public void holdForShot(boolean hold) {
        holdingForShot = hold && enabled;
        if (holdingForShot) {
            turret.hold();
        }
    }

    /** Distance to the field target; 0 when unknown or aiming at a fixed relative preset. */
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
