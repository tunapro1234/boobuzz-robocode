package boobuzz.core.subsystem.stub;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.IShooter;
import boobuzz.core.subsystem.hood.PairedHood;

import java.util.Map;

/**
 * Timing-only shooter used until the real shooter mechanism is integrated. The composed
 * hood is the real paired-servo mapping; its settle status is an estimate, not feedback.
 */
public final class StubShooter implements IShooter {

    private final PairedHood hood = new PairedHood();

    private double targetRpm;
    private long spinUpStartedMs;
    private long nowMs;
    private long feedEndMs;
    private boolean spinning;
    private double openLoopPower;
    private double feederPower;
    private boolean feeding;
    private boolean feedStartPending;
    private boolean feedEndPending;

    @Override
    public void observe(RobotState state) {
        nowMs = state.t();
        hood.observe(state);
        finishFeedIfDue();
    }

    @Override
    public void spinUp(double rpm) {
        if (!Double.isFinite(rpm) || rpm <= 0.0) {
            spinDown();
            return;
        }
        openLoopPower = 0.0;
        if (!spinning || Double.doubleToLongBits(targetRpm) != Double.doubleToLongBits(rpm)) {
            targetRpm = rpm;
            spinUpStartedMs = nowMs;
            spinning = true;
        }
    }

    @Override
    public void spinDown() {
        targetRpm = 0.0;
        openLoopPower = 0.0;
        spinning = false;
        if (feeding) {
            feeding = false;
            feedEndPending = true;
        }
        feedStartPending = false;
    }

    @Override
    public boolean isReady() {
        return spinning && nowMs - spinUpStartedMs >= secondsToMillis(RobotConstants.STUB_SPINUP_S);
    }

    @Override
    public void feed() {
        if (isReady() && !feeding) {
            feeding = true;
            feedEndMs = nowMs + secondsToMillis(RobotConstants.STUB_FEED_S);
            feedStartPending = true;
        }
    }

    @Override
    public boolean isFeeding() {
        finishFeedIfDue();
        return feeding;
    }

    /** Timing-only stub: records the power; there is no flywheel speed to guard. */
    @Override
    public void runOpenLoop(double power) {
        spinDown();
        openLoopPower = Double.isFinite(power) ? power : 0.0;
    }

    @Override
    public void setFeederPower(double power) {
        if (feeding) {
            feeding = false;
            feedEndPending = true;
        }
        feedStartPending = false;
        feederPower = Double.isFinite(power) ? power : 0.0;
    }

    /** No speed sensor: stopped whenever nothing is commanded to spin. */
    @Override
    public boolean isStopped() {
        return !spinning && openLoopPower == 0.0;
    }

    public double openLoopPower() {
        return openLoopPower;
    }

    public double feederPower() {
        return feederPower;
    }

    @Override
    public void setHoodAngleDeg(double angleDeg) {
        hood.setAngleDeg(angleDeg);
    }

    @Override
    public boolean hoodSettled() {
        return hood.settled();
    }

    @Override
    public void update(RobotAction.Builder out) {
        hood.update(out);
        if (feedStartPending) {
            out.event("shooter.feed.start", nowMs, java.util.Collections.emptyMap());
            feedStartPending = false;
        }
        if (feedEndPending) {
            out.event("shooter.feed.end", nowMs, java.util.Collections.emptyMap());
            feedEndPending = false;
        }
    }

    private void finishFeedIfDue() {
        if (feeding && nowMs >= feedEndMs) {
            feeding = false;
            feedEndPending = true;
        }
    }

    private static long secondsToMillis(double seconds) {
        return Math.max(1L, Math.round(seconds * 1000.0));
    }
}
