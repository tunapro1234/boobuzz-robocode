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
    private AimResult aimStatus = AimResult.ACCEPTED;

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
            aimStatus = AimResult.INVALID_INPUT;
            scan();
            return;
        }
        aimStatus = AimResult.ACCEPTED;
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

    /** Timing stub: accepts +-90 deg like the real turret and settles after the stub delay. */
    @Override
    public AimResult aimRelative(double angleRad) {
        if (!Double.isFinite(angleRad)) {
            aimStatus = AimResult.INVALID_INPUT;
            return aimStatus;
        }
        double deg = Math.toDegrees(angleRad);
        if (deg < RobotConstants.TURRET_MIN_DEG || deg > RobotConstants.TURRET_MAX_DEG) {
            aimStatus = AimResult.OUT_OF_RANGE;
            return aimStatus;
        }
        if (!aiming || scanning || Double.doubleToLongBits(this.angleRad)
                != Double.doubleToLongBits(angleRad)) {
            this.angleRad = angleRad;
            aimStartedMs = nowMs;
            aiming = true;
            scanning = false;
            lockedEventPending = false;
        }
        aimStatus = AimResult.ACCEPTED;
        return aimStatus;
    }

    @Override
    public AimResult aimStatus() {
        return aimStatus;
    }

    @Override
    public void disable() {
        hold();
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
