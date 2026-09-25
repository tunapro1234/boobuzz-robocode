package boobuzz.core.subsystem.hood;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.ISubsystem;

/**
 * One hood driven by two mirrored position servos (archive HoodSubsystem).
 *
 * <p>Both servo commands come from one clipped hood angle in the same action. There is
 * no angle sensor: the reported angle is the last command and "settled" is a time
 * estimate from the fixture rate. Nothing is written until the first explicit setpoint,
 * so init never moves the hood.
 */
public final class PairedHood implements ISubsystem {

    // Unknown initial position budgets worst-case full travel.
    private static final double UNKNOWN_TRAVEL_DEG =
            RobotConstants.HOOD_MAX_DEG - RobotConstants.HOOD_MIN_DEG;

    private boolean commanded;
    private boolean pendingStart;
    private double targetDeg;
    private long nowMs;

    // Current move estimate. Before the first settled move the start angle is unknown.
    private boolean hasMove;
    private boolean moveStartKnown;
    private double moveFromDeg;
    private double moveToDeg;
    private long moveStartMs;
    private long settleAtMs;

    @Override
    public void observe(RobotState state) {
        nowMs = state.t();
    }

    /** Command the hood angle; values outside 25..50 deg are clipped like the archive. */
    public void setAngleDeg(double angleDeg) {
        if (!Double.isFinite(angleDeg)) {
            throw new IllegalArgumentException("hood angle must be finite: " + angleDeg);
        }
        double clipped = clip(angleDeg);
        if (commanded && clipped == targetDeg) {
            return;
        }
        targetDeg = clipped;
        pendingStart = true;
        commanded = true;
    }

    /** Last commanded angle, or NaN before the first command (no measured angle exists). */
    public double commandedAngleDeg() {
        return commanded ? targetDeg : Double.NaN;
    }

    public boolean isCommanded() {
        return commanded;
    }

    /** True once the estimated travel time plus margin has passed since the last command. */
    public boolean settled() {
        return commanded && !pendingStart && nowMs >= settleAtMs;
    }

    @Override
    public void update(RobotAction.Builder out) {
        if (!commanded) {
            return;
        }
        if (pendingStart) {
            startMove();
        }
        double u = servoFraction(targetDeg);
        out.servo(RobotConstants.HOOD_LEFT_SERVO_NAME, 1.0 - u);
        out.servo(RobotConstants.HOOD_RIGHT_SERVO_NAME, u);
    }

    private void startMove() {
        if (hasMove && nowMs >= settleAtMs) {
            moveFromDeg = moveToDeg;
            moveStartKnown = true;
        } else if (hasMove && moveStartKnown) {
            moveFromDeg = interpolated();
        } else {
            moveStartKnown = false;
        }
        double travel = moveStartKnown ? Math.abs(targetDeg - moveFromDeg) : UNKNOWN_TRAVEL_DEG;
        moveToDeg = targetDeg;
        moveStartMs = nowMs;
        settleAtMs = nowMs + travelMs(travel) + RobotConstants.HOOD_SETTLE_MARGIN_MS;
        hasMove = true;
        pendingStart = false;
    }

    private double interpolated() {
        double distance = moveToDeg - moveFromDeg;
        double moved = RobotConstants.HOOD_FIXTURE_RATE_DEG_S * (nowMs - moveStartMs) / 1000.0;
        return moved >= Math.abs(distance) ? moveToDeg : moveFromDeg + Math.signum(distance) * moved;
    }

    /** Estimated settle time of the current move, for tests and traces. */
    public long settleAtMs() {
        return settleAtMs;
    }

    /** Archive mapping: fraction of the 300-deg servo range for a hood angle (right servo). */
    public static double servoFraction(double angleDeg) {
        double t = (clip(angleDeg) - RobotConstants.HOOD_MIN_DEG)
                / (RobotConstants.HOOD_MAX_DEG - RobotConstants.HOOD_MIN_DEG);
        return t * RobotConstants.HOOD_SERVO_TRAVEL_DEG / RobotConstants.HOOD_SERVO_RANGE_DEG;
    }

    private static long travelMs(double travelDeg) {
        return (long) Math.ceil(travelDeg / RobotConstants.HOOD_FIXTURE_RATE_DEG_S * 1000.0);
    }

    private static double clip(double angleDeg) {
        return Math.max(RobotConstants.HOOD_MIN_DEG, Math.min(RobotConstants.HOOD_MAX_DEG, angleDeg));
    }
}
