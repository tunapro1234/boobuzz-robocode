package boobuzz.core.subsystem.stub;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.ITurret;

import java.util.Map;

/** Timing-only turret stub used until the real turret mechanism is integrated. */
public final class StubTurret implements ITurret {

    private double targetX;
    private double targetY;
    private double angleRad;
    private long nowMs;
    private long aimStartedMs;
    private boolean aiming;
    private boolean scanning;
    private boolean lockedEventPending;
    private boolean scanEventPending;

    @Override
    public void observe(RobotState state) {
        nowMs = state.t();
        if (aiming && !lockedEventPending && onTarget()) {
            lockedEventPending = true;
        }
    }

    @Override
    public void aimAt(double fieldX, double fieldY) {
        if (!Double.isFinite(fieldX) || !Double.isFinite(fieldY)) {
            scan();
            return;
        }
        if (!aiming || scanning || Double.doubleToLongBits(targetX)
                != Double.doubleToLongBits(fieldX)
                || Double.doubleToLongBits(targetY) != Double.doubleToLongBits(fieldY)) {
            targetX = fieldX;
            targetY = fieldY;
            angleRad = Math.atan2(fieldY, fieldX);
            aimStartedMs = nowMs;
            aiming = true;
            scanning = false;
            lockedEventPending = false;
        }
    }

    @Override
    public void scan() {
        if (!scanning) {
            scanning = true;
            aiming = false;
            scanEventPending = true;
        }
    }

    @Override
    public void hold() {
        // Holding cancels an in-flight aim/scan transition. Keep angleRad as the
        // physical hold position, but do not report a later lock for old work.
        aiming = false;
        scanning = false;
        lockedEventPending = false;
        scanEventPending = false;
    }

    @Override
    public boolean onTarget() {
        return aiming && nowMs - aimStartedMs
                >= Math.round(RobotConstants.STUB_TURRET_SETTLE_S * 1000.0);
    }

    @Override
    public double angleRad() {
        return angleRad;
    }

    @Override
    public void update(RobotAction.Builder out) {
        if (scanEventPending) {
            out.event("turret.scan", nowMs, java.util.Collections.emptyMap());
            scanEventPending = false;
        }
        if (lockedEventPending) {
            out.event("turret.locked", nowMs, java.util.Collections.emptyMap());
            lockedEventPending = false;
        }
    }
}
