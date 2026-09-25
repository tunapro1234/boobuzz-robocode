package boobuzz.core.subsystem.turret;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.ITurret;

import com.pedropathing.math.Pose;

import java.util.ArrayDeque;
import java.util.Collections;
import java.util.Iterator;
import java.util.Map;

/**
 * ONE turret over two CR servos (archive TurretPidPazarSubsystem): one angle estimate,
 * one PID, the same logical power on both outputs.
 *
 * <p>Angle estimate: the shared shooterLeft encoder (read only, never reset or driven
 * here) integrates motion; the absolute analog seeds it and is blended in through the
 * archive's Kalman filter during a 0.75 s full-trust + 1.25 s fade calibration window.
 * Both outputs are zero while calibrating. After calibration the analog is not used.
 *
 * <p>Explicit differences from the archive, all on the safe side:
 * <ul>
 *   <li>Time is HAL ms, not wall clock.</li>
 *   <li>Snap-to-zero is disabled (spec B07 v0) so a real offset is never concealed.</li>
 *   <li>A missing/out-of-range analog at startup, or an initial angle outside the hard
 *       range, keeps the turret uninitialized (outputs 0, aims NOT_INITIALIZED) until a
 *       valid reading arrives; the archive assumed the reading was valid.</li>
 *   <li>A missing encoder, or a jump larger than the whole hard range in one tick,
 *       zeroes both outputs and restarts calibration from the analog.</li>
 *   <li>Out-of-range aims are rejected and the previous target is kept; the archive
 *       clamped them to the limit and reported nothing.</li>
 *   <li>Within {@link RobotConstants#TURRET_SOFT_MARGIN_DEG} of a hard stop, power
 *       toward that stop fades linearly to 0 at the stop (spec fixture soft margin).</li>
 * </ul>
 */
public final class PairedCrTurret implements ITurret {

    private enum Mode { DISABLED, HOLD, RELATIVE, FIELD }

    private static final double DEGREES_PER_TICK = 360.0 * RobotConstants.TURRET_ENCODER_GEAR_TEETH
            / (RobotConstants.TURRET_GEAR_TEETH * RobotConstants.TURRET_ENCODER_TICKS_PER_REV);
    private static final double RANGE_DEG =
            RobotConstants.TURRET_MAX_DEG - RobotConstants.TURRET_MIN_DEG;
    private static final long CALIBRATION_MS = Math.round(
            (RobotConstants.TURRET_FULL_TRUST_S + RobotConstants.TURRET_FADE_OUT_S) * 1000.0);

    // Latest observation.
    private long nowMs;
    private Integer encoderTicks;
    private Double analogVolts;
    private Pose pose;

    // Estimator.
    private boolean calibrating;
    private boolean initialized;
    private long calibrationStartMs;
    private int lastTicks;
    private double kalmanX;
    private double kalmanP;
    private boolean lpfInitialized;
    private double lpfDeg;
    private boolean snapChecked;

    // Rate estimate for the settled criterion, measured over the settle window: one
    // encoder tick per 20 ms loop is already ~3 deg/s, so a per-tick rate is quantized.
    private final ArrayDeque<double[]> angleHistory = new ArrayDeque<>();
    private double rateDegS;

    // Command.
    private Mode mode = Mode.DISABLED;
    private boolean hasTarget;
    private double targetDeg;
    private double fieldX;
    private double fieldY;
    private AimResult aimStatus = AimResult.NOT_INITIALIZED;

    // Archive FTCLib PID state.
    private final ArchivePid pid = new ArchivePid();
    private boolean pidFresh = true;
    private long lastPidMs;
    private double appliedPower;

    // Settle / events.
    private boolean withinSettle;
    private long withinSinceMs;
    private boolean wasOnTarget;
    private boolean readyEventPending;
    private boolean faultEventPending;
    private boolean faulted;

    @Override
    public void observe(RobotState state) {
        nowMs = state.t();
        pose = state.pinpoint();
        encoderTicks = state.enc().get(RobotConstants.TURRET_ENCODER_NAME);
        analogVolts = state.analogVolts().get(RobotConstants.TURRET_ANALOG_NAME);
        updateEstimate();
        updateRate();
    }

    // ---- estimator -------------------------------------------------------------------

    private void updateEstimate() {
        if (encoderTicks == null) {
            fault();
            return;
        }
        if (!calibrating && !initialized) {
            startCalibration();
            return;
        }
        int delta = encoderTicks - lastTicks;
        lastTicks = encoderTicks;
        double deltaDeg = ticksToDegrees(delta);
        if (Math.abs(deltaDeg) > RANGE_DEG) {
            // Physically impossible in one tick: the encoder was reset or glitched.
            fault();
            startCalibration();
            return;
        }
        double predictedX = kalmanX + deltaDeg;
        double predictedP = kalmanP + RobotConstants.TURRET_KALMAN_Q;
        if (!calibrating) {
            kalmanX = normalizeTo180(predictedX);
            kalmanP = Math.min(predictedP, 1.0);
            return;
        }

        double elapsedS = (nowMs - calibrationStartMs) / 1000.0;
        if (!snapChecked && elapsedS >= RobotConstants.TURRET_FULL_TRUST_S) {
            snapChecked = true;
            if (RobotConstants.TURRET_SNAP_TO_ZERO_ENABLED
                    && Math.abs(predictedX) < RobotConstants.TURRET_SNAP_TO_ZERO_THRESHOLD_DEG) {
                predictedX = 0.0;
            }
        }
        if (nowMs - calibrationStartMs >= CALIBRATION_MS) {
            calibrating = false;
            initialized = true;
            readyEventPending = true;
            kalmanX = normalizeTo180(predictedX);
            kalmanP = Math.min(predictedP, 1.0);
            return;
        }
        Double analogDeg = analogAngleDeg(analogVolts);
        if (analogDeg == null) {
            // A calibration sample went missing: never blend a guess, start over.
            fault();
            startCalibration();
            return;
        }
        double filtered = filterAnalog(analogDeg);
        double trust = elapsedS < RobotConstants.TURRET_FULL_TRUST_S ? 1.0
                : 1.0 - clamp((elapsedS - RobotConstants.TURRET_FULL_TRUST_S)
                        / RobotConstants.TURRET_FADE_OUT_S, 0.0, 1.0);
        double gain = predictedP / (predictedP + RobotConstants.TURRET_KALMAN_R) * trust;
        kalmanX = normalizeTo180(predictedX + gain * angleDifference(predictedX, filtered));
        kalmanP = (1.0 - gain) * predictedP;
    }

    private void startCalibration() {
        calibrating = false;
        initialized = false;
        Double analogDeg = analogAngleDeg(analogVolts);
        if (encoderTicks == null || analogDeg == null
                || analogDeg < RobotConstants.TURRET_MIN_DEG
                || analogDeg > RobotConstants.TURRET_MAX_DEG) {
            fault();
            return;
        }
        faulted = false;
        calibrating = true;
        calibrationStartMs = nowMs;
        lastTicks = encoderTicks;
        kalmanX = analogDeg;
        kalmanP = RobotConstants.TURRET_KALMAN_R;
        lpfInitialized = false;
        filterAnalog(analogDeg);
        snapChecked = false;
        angleHistory.clear();
    }

    private void fault() {
        if (!faulted) {
            faultEventPending = true;
        }
        faulted = true;
        calibrating = false;
        initialized = false;
        angleHistory.clear();
        resetPid();
        // The old target was relative to an estimate that is gone: never resume toward it.
        // Field tracking re-evaluates after recalibration; a fixed aim degrades to hold.
        hasTarget = false;
        if (mode == Mode.RELATIVE) {
            mode = Mode.HOLD;
        }
    }

    private double filterAnalog(double rawDeg) {
        if (!lpfInitialized) {
            lpfDeg = rawDeg;
            lpfInitialized = true;
            return lpfDeg;
        }
        lpfDeg = normalizeTo180(lpfDeg
                + RobotConstants.TURRET_ANALOG_LPF_ALPHA * angleDifference(lpfDeg, rawDeg));
        return lpfDeg;
    }

    /**
     * Archive conversion: volts over 0..3.3 V -> shaft degrees, minus the 125 deg shaft
     * offset, times the encoder/turret gear ratio. Null when missing or out of range.
     * Over a full shaft turn only this branch can land in the +-90 deg hard range (the
     * archive's second "+360" option is always above +90 deg with this gearing).
     */
    public static Double analogAngleDeg(Double volts) {
        if (volts == null || !Double.isFinite(volts)
                || volts < RobotConstants.TURRET_ANALOG_MIN_V
                || volts > RobotConstants.TURRET_ANALOG_MAX_V) {
            return null;
        }
        double shaftDeg = (volts - RobotConstants.TURRET_ANALOG_MIN_V)
                / (RobotConstants.TURRET_ANALOG_MAX_V - RobotConstants.TURRET_ANALOG_MIN_V) * 360.0;
        return (shaftDeg - RobotConstants.TURRET_ANALOG_SHAFT_OFFSET_DEG)
                * RobotConstants.TURRET_ENCODER_GEAR_TEETH / RobotConstants.TURRET_GEAR_TEETH;
    }

    /** Archive sign/gear: encoder reversed, 360 / (0.715 * 8192) degrees per tick. */
    public static double ticksToDegrees(double ticks) {
        return (RobotConstants.TURRET_ENCODER_REVERSED ? -ticks : ticks) * DEGREES_PER_TICK;
    }

    private void updateRate() {
        if (!initialized) {
            angleHistory.clear();
            rateDegS = 0.0;
            return;
        }
        double[] last = angleHistory.peekLast();
        if (last == null || nowMs > (long) last[0]) {
            angleHistory.addLast(new double[] {nowMs, kalmanX});
        }
        // Keep the newest sample at or before now - window as the rate baseline.
        while (angleHistory.size() > 2) {
            Iterator<double[]> it = angleHistory.iterator();
            it.next();
            if (nowMs - (long) it.next()[0] >= RobotConstants.TURRET_SETTLE_MS) {
                angleHistory.removeFirst();
            } else {
                break;
            }
        }
        double[] first = angleHistory.peekFirst();
        long spanMs = nowMs - (long) first[0];
        rateDegS = spanMs > 0 ? angleDifference(first[1], kalmanX) / (spanMs / 1000.0) : 0.0;
    }

    // ---- commands --------------------------------------------------------------------

    @Override
    public void aimAt(double fieldX, double fieldY) {
        if (!Double.isFinite(fieldX) || !Double.isFinite(fieldY)) {
            aimStatus = AimResult.INVALID_INPUT;
            return;
        }
        if (!initialized) {
            aimStatus = AimResult.NOT_INITIALIZED;
            return;
        }
        this.fieldX = fieldX;
        this.fieldY = fieldY;
        enterAimMode(Mode.FIELD);
        evaluateField();
    }

    @Override
    public AimResult aimRelative(double angleRad) {
        if (!Double.isFinite(angleRad)) {
            aimStatus = AimResult.INVALID_INPUT;
            return aimStatus;
        }
        if (!initialized) {
            aimStatus = AimResult.NOT_INITIALIZED;
            return aimStatus;
        }
        double deg = Math.toDegrees(angleRad);
        if (!inRange(deg)) {
            aimStatus = AimResult.OUT_OF_RANGE;
            return aimStatus;
        }
        enterAimMode(Mode.RELATIVE);
        setTarget(deg);
        aimStatus = AimResult.ACCEPTED;
        return aimStatus;
    }

    @Override
    public AimResult aimStatus() {
        return aimStatus;
    }

    /** The real turret has no sweep; scan is a hold (the stub's scan is test-only). */
    @Override
    public void scan() {
        hold();
    }

    /**
     * Freeze ownership: field tracking stops and the turret holds its current target, or
     * its current angle when it had none. Repeated calls keep the same hold target.
     */
    @Override
    public void hold() {
        if (mode == Mode.HOLD && hasTarget) {
            return;
        }
        if (mode == Mode.DISABLED) {
            hasTarget = false;
            resetPid();
        }
        mode = Mode.HOLD;
        if (!hasTarget && initialized) {
            setTarget(clamp(kalmanX, RobotConstants.TURRET_MIN_DEG, RobotConstants.TURRET_MAX_DEG));
        }
        if (initialized) {
            aimStatus = AimResult.ACCEPTED;
        }
    }

    @Override
    public void disable() {
        mode = Mode.DISABLED;
        hasTarget = false;
        appliedPower = 0.0;
        resetPid();
    }

    private void enterAimMode(Mode next) {
        if (mode == Mode.DISABLED) {
            resetPid();
        }
        mode = next;
    }

    private void evaluateField() {
        if (pose == null || !Double.isFinite(pose.x()) || !Double.isFinite(pose.y())
                || !Double.isFinite(pose.heading())) {
            aimStatus = AimResult.INVALID_INPUT;
            return;
        }
        double relDeg = relativeBearingDeg(pose, fieldX, fieldY);
        if (!inRange(relDeg)) {
            aimStatus = AimResult.OUT_OF_RANGE;
            return;
        }
        setTarget(relDeg);
        aimStatus = AimResult.ACCEPTED;
    }

    /** Archive AimingController geometry: bearing minus heading, CCW positive, no offset. */
    public static double relativeBearingDeg(Pose robot, double fieldX, double fieldY) {
        double bearing = Math.atan2(fieldY - robot.y(), fieldX - robot.x());
        return normalizeTo180(Math.toDegrees(bearing - robot.heading()));
    }

    private void setTarget(double deg) {
        // Settling is judged on error and rate only, so a slowly moving field target
        // (robot driving) can still lock.
        targetDeg = deg;
        hasTarget = true;
    }

    // ---- control ---------------------------------------------------------------------

    @Override
    public void update(RobotAction.Builder out) {
        if (!initialized && mode != Mode.DISABLED) {
            aimStatus = AimResult.NOT_INITIALIZED;
        }
        if (initialized && mode == Mode.FIELD) {
            evaluateField();
        }
        if (initialized && mode == Mode.HOLD && !hasTarget) {
            hold();
        }
        if (!initialized || mode == Mode.DISABLED || !hasTarget) {
            appliedPower = 0.0;
            resetPid();
        } else {
            closedLoop();
        }
        out.motor(RobotConstants.TURRET_PRIMARY_SERVO_NAME, appliedPower);
        out.motor(RobotConstants.TURRET_SECONDARY_SERVO_NAME, appliedPower);
        emitEvents(out);
    }

    private void closedLoop() {
        double error = targetDeg - kalmanX;
        double periodS = 0.0;
        if (!pidFresh) {
            periodS = (nowMs - lastPidMs) / 1000.0;
        }
        pidFresh = false;
        lastPidMs = nowMs;
        double command = pid.calculate(error, kalmanX, periodS);
        command = clamp(command, -RobotConstants.TURRET_MAX_POWER, RobotConstants.TURRET_MAX_POWER);
        appliedPower = softLimit(command, kalmanX);
        updateSettle(Math.abs(error) <= RobotConstants.TURRET_SETTLE_TOL_DEG
                && Math.abs(rateDegS) <= RobotConstants.TURRET_SETTLE_RATE_DEG_S);
    }

    /** Positive power increases the angle (archive PID drives error directly). */
    static double softLimit(double power, double angleDeg) {
        double margin = RobotConstants.TURRET_SOFT_MARGIN_DEG;
        if (power > 0.0) {
            return power * clamp((RobotConstants.TURRET_MAX_DEG - angleDeg) / margin, 0.0, 1.0);
        }
        if (power < 0.0) {
            return power * clamp((angleDeg - RobotConstants.TURRET_MIN_DEG) / margin, 0.0, 1.0);
        }
        return 0.0;
    }

    private void updateSettle(boolean within) {
        if (within && !withinSettle) {
            withinSinceMs = nowMs;
        }
        withinSettle = within;
    }

    private void resetPid() {
        pid.reset();
        pidFresh = true;
        withinSettle = false;
    }

    @Override
    public boolean onTarget() {
        return initialized && mode != Mode.DISABLED && hasTarget
                && aimStatus == AimResult.ACCEPTED && withinSettle
                && nowMs - withinSinceMs >= RobotConstants.TURRET_SETTLE_MS;
    }

    /** Current estimate in radians (CCW positive), or NaN before the first valid seed. */
    @Override
    public double angleRad() {
        return initialized || calibrating ? Math.toRadians(kalmanX) : Double.NaN;
    }

    private void emitEvents(RobotAction.Builder out) {
        if (faultEventPending) {
            out.event("turret.fault", nowMs, Collections.emptyMap());
            faultEventPending = false;
        }
        if (readyEventPending) {
            out.event("turret.ready", nowMs, Map.of("angleDeg", kalmanX));
            readyEventPending = false;
        }
        boolean on = onTarget();
        if (on && !wasOnTarget) {
            out.event("turret.locked", nowMs, Map.of("angleDeg", kalmanX));
        }
        wasOnTarget = on;
    }

    // ---- accessors for tests and traces ----------------------------------------------

    public boolean isInitialized() {
        return initialized;
    }

    public boolean isCalibrating() {
        return calibrating;
    }

    public double angleDeg() {
        return kalmanX;
    }

    public double targetDeg() {
        return hasTarget ? targetDeg : Double.NaN;
    }

    public double appliedPower() {
        return appliedPower;
    }

    public double rateDegS() {
        return rateDegS;
    }

    // ---- math ------------------------------------------------------------------------

    private static boolean inRange(double deg) {
        return deg >= RobotConstants.TURRET_MIN_DEG && deg <= RobotConstants.TURRET_MAX_DEG;
    }

    static double angleDifference(double from, double to) {
        double d = to - from;
        while (d > 180.0) {
            d -= 360.0;
        }
        while (d < -180.0) {
            d += 360.0;
        }
        return d;
    }

    static double normalizeTo180(double angle) {
        double a = angle % 360.0;
        if (a > 180.0) {
            a -= 360.0;
        }
        if (a < -180.0) {
            a += 360.0;
        }
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /**
     * FTCLib 2.1.1 PIDFController as the archive used it: derivative on measurement
     * (setSetPoint quirk), totalError integrated with the period and clamped to
     * +-{@link RobotConstants#TURRET_INTEGRAL_MAX}, no derivative on a zero period.
     */
    static final class ArchivePid {
        private double previousMeasurement;
        private double totalError;

        double calculate(double error, double measurement, double periodS) {
            double derivative = Math.abs(periodS) > 1e-6
                    ? -(measurement - previousMeasurement) / periodS : 0.0;
            previousMeasurement = measurement;
            totalError = clamp(totalError + periodS * error,
                    -RobotConstants.TURRET_INTEGRAL_MAX, RobotConstants.TURRET_INTEGRAL_MAX);
            return RobotConstants.TURRET_KP * error + RobotConstants.TURRET_KI * totalError
                    + RobotConstants.TURRET_KD * derivative;
        }

        void reset() {
            previousMeasurement = 0.0;
            totalError = 0.0;
        }

        double totalError() {
            return totalError;
        }
    }
}
