package boobuzz.core.subsystem.stub;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.IShooter;

import java.util.Map;

/** Timing-only shooter used until the real shooter mechanism is integrated. */
public final class StubShooter implements IShooter {

    private double targetRpm;
    private long spinUpStartedMs;
    private long nowMs;
    private long feedEndMs;
    private boolean spinning;
    private boolean feeding;
    private boolean feedStartPending;
    private boolean feedEndPending;

    @Override
    public void observe(RobotState state) {
        nowMs = state.t();
        finishFeedIfDue();
    }

    @Override
    public void spinUp(double rpm) {
        if (!Double.isFinite(rpm) || rpm <= 0.0) {
            spinDown();
            return;
        }
        if (!spinning || Double.doubleToLongBits(targetRpm) != Double.doubleToLongBits(rpm)) {
            targetRpm = rpm;
            spinUpStartedMs = nowMs;
            spinning = true;
        }
    }

    @Override
    public void spinDown() {
        targetRpm = 0.0;
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

    @Override
    public void update(RobotAction.Builder out) {
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
