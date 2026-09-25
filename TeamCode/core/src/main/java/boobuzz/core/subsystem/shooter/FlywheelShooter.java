package boobuzz.core.subsystem.shooter;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.IShooter;
import boobuzz.core.subsystem.feeder.PulseFeeder;
import boobuzz.core.subsystem.hood.PairedHood;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * ONE shooter over two flywheel motors (archive ShooterPidfPowerSubsystem), plus the
 * composed feeder and hood. Facade parts are observed/updated here once and are never
 * registered as separate subsystems.
 *
 * <p>One controller, one speed source: shooterRight velocity. Output right = p,
 * left = p * followerScale; motor reversal is applied once by the HAL profile.
 *
 * <p>Explicit differences from the archive, all on the safe side:
 * <ul>
 *   <li>Time is HAL ms. After a reset (enable, tuning change, invalid sensor) the first
 *       closed-loop tick has no derivative and no integral step; the archive used a
 *       stale wall-clock dt and an error of 0 there. A >50 RPM target change resets
 *       only readiness, exactly like the archive (ShooterPidfPowerSubsystem:64-68);
 *       integral and previous error are otherwise cleared only on disable (:79-86).</li>
 *   <li>Deliberate non-archive addition: an accepted tuning change clears the
 *       controller and readiness (the archive's tuning was static).</li>
 *   <li>Deliberate non-archive addition: a missing or non-finite velocity zeroes both
 *       outputs and clears the controller and readiness until the sensor returns (the
 *       archive had no sensor-loss concept).</li>
 *   <li>Spin-down also stops the feeder, so nothing is fed with the flywheel off.</li>
 *   <li>A zero, negative or non-finite target spins down (outputs 0); the archive kept
 *       the loop closed at 0 RPM and accepted negative targets.</li>
 *   <li>Readiness also requires the CURRENT observation to be valid and within
 *       tolerance. Logic calls {@link #feed()} after observe but before update, so the
 *       dwell flag alone would be one tick stale; the archive ran periodic() first.</li>
 *   <li>Spin-down before reverse (spec B08 safety correction to the archive's instant
 *       +-1 jam toggle): power is never applied against a measured rotation outside the
 *       readiness tolerance. Open loop coasts until stopped; the closed loop never drives
 *       a backwards-spinning wheel forward. Closed-loop braking of a forward wheel stays
 *       archive behavior.</li>
 * </ul>
 * Anti-windup is the archive's only: integral zone reset plus accumulator clamp; the
 * output clip does not stop integration.
 */
public final class FlywheelShooter implements IShooter {

    private final Supplier<ShooterTuning> tuningSupplier;
    private final PulseFeeder feeder = new PulseFeeder();
    private final PairedHood hood = new PairedHood();

    private ShooterTuning tuning;
    private ShooterTuning lastRejected;
    private int rejectedTuningCount;
    private boolean tuningEventPending;
    private boolean rejectEventPending;

    private long nowMs;
    private boolean sensorValid;
    private double measuredRpm = Double.NaN;

    private boolean enabled;
    private boolean openLoop;
    private double openLoopPower;
    private double targetRpm;
    private boolean fresh = true;
    private long lastLoopMs;
    private double previousErrorRpm;
    private double integralAccum;
    private boolean stable;
    private boolean previouslyWithinTolerance;
    private long withinSinceMs;
    private double appliedPower;
    private double lastPidPower;
    private double lastFfPower;

    private boolean feedPending;
    private boolean wasPulsing;

    public FlywheelShooter() {
        this(() -> ShooterTuning.DEFAULTS);
    }

    /** {@code tuningSupplier} is read exactly once per tick (live dashboard tuning). */
    public FlywheelShooter(Supplier<ShooterTuning> tuningSupplier) {
        this.tuningSupplier = Objects.requireNonNull(tuningSupplier, "tuningSupplier");
    }

    /** Archive conversion: wheel RPM = ticks/s * 60 / ticksPerRev * motorToWheel. */
    public static double wheelRpmFromTicksPerSecond(double ticksPerSecond) {
        return ticksPerSecond * 60.0 / RobotConstants.SHOOTER_RIGHT.ticksPerRev()
                * RobotConstants.SHOOTER_MOTOR_TO_WHEEL;
    }

    @Override
    public void observe(RobotState state) {
        nowMs = state.t();
        hood.observe(state);
        feeder.observe(state);
        Double velocity = state.vel().get(RobotConstants.SHOOTER_FEEDBACK_ENCODER_NAME);
        sensorValid = velocity != null && Double.isFinite(velocity);
        measuredRpm = sensorValid ? wheelRpmFromTicksPerSecond(velocity) : Double.NaN;
    }

    @Override
    public void spinUp(double rpm) {
        if (!Double.isFinite(rpm) || rpm <= 0.0) {
            spinDown();
            return;
        }
        if (!enabled) {
            resetController();
        } else if (Math.abs(rpm - targetRpm) > RobotConstants.SHOOTER_TARGET_RESET_RPM) {
            // Archive: a target change resets only the readiness dwell; the integral and
            // previous error carry over so a moving aim target does not erase them.
            stable = false;
            previouslyWithinTolerance = false;
        }
        enabled = true;
        openLoop = false;
        openLoopPower = 0.0;
        targetRpm = rpm;
    }

    @Override
    public void spinDown() {
        enabled = false;
        openLoop = false;
        openLoopPower = 0.0;
        targetRpm = 0.0;
        resetController();
        appliedPower = 0.0;
        feedPending = false;
        feeder.stop();
    }

    @Override
    public boolean isReady() {
        return enabled && stable && sensorValid
                && Math.abs(targetRpm - measuredRpm) <= activeTuning().toleranceRpm();
    }

    /** One archive pulse, only when ready and the feeder is idle; never queued. */
    @Override
    public void feed() {
        if (isReady() && !feedPending && feeder.phase() == PulseFeeder.Phase.IDLE
                && !feeder.isRunning()) {
            feedPending = true;
        }
    }

    @Override
    public boolean isFeeding() {
        return feedPending || feeder.isPulsing();
    }

    @Override
    public void runOpenLoop(double power) {
        if (enabled) {
            enabled = false;
            targetRpm = 0.0;
            resetController();
        }
        feedPending = false;
        openLoop = true;
        openLoopPower = Double.isFinite(power) ? RobotAction.clamp(power, -1.0, 1.0) : 0.0;
    }

    @Override
    public void setFeederPower(double power) {
        feedPending = false;
        feeder.stop();
        feeder.setPower(power);
    }

    @Override
    public boolean isStopped() {
        return sensorValid && Math.abs(measuredRpm) <= activeTuning().toleranceRpm();
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
        readTuning();

        if (feedPending) {
            feeder.requestPulse();
        }
        feeder.update(out);
        if (feedPending) {
            // The pulse has started; clearing the request lets it end after one pulse.
            feeder.clearRequest();
            feedPending = false;
        }

        if (openLoop) {
            stable = false;
            appliedPower = againstRotation(openLoopPower) ? 0.0 : openLoopPower;
        } else if (!enabled) {
            stable = false;
            appliedPower = 0.0;
        } else if (!sensorValid) {
            // Deliberate non-archive addition: the archive had no sensor-loss concept.
            resetController();
            appliedPower = 0.0;
        } else {
            closedLoop();
            if (appliedPower > 0.0 && measuredRpm < -activeTuning().toleranceRpm()) {
                appliedPower = 0.0;
            }
        }
        out.motor(RobotConstants.SHOOTER_RIGHT_MOTOR_NAME, appliedPower);
        out.motor(RobotConstants.SHOOTER_LEFT_MOTOR_NAME,
                appliedPower * RobotConstants.SHOOTER_FOLLOWER_SCALE);

        emitEvents(out);
    }

    /** Open-loop power that would oppose the measured rotation (or runs blind). */
    private boolean againstRotation(double power) {
        if (power == 0.0) {
            return false;
        }
        if (!sensorValid) {
            return true;
        }
        return Math.abs(measuredRpm) > activeTuning().toleranceRpm()
                && Math.signum(power) != Math.signum(measuredRpm);
    }

    private void closedLoop() {
        double error = targetRpm - measuredRpm;
        double derivative = 0.0;
        if (!fresh) {
            double dt = Math.max((nowMs - lastLoopMs) / 1000.0, RobotConstants.SHOOTER_MIN_DT_S);
            updateIntegral(error, dt);
            derivative = (error - previousErrorRpm) / dt;
        }
        fresh = false;
        lastLoopMs = nowMs;
        previousErrorRpm = error;

        double pidPower = tuning.kP() * error + tuning.kI() * integralAccum
                + tuning.kD() * derivative;
        double ffPower = feedforwardPower();
        appliedPower = RobotAction.clamp(pidPower + ffPower, -1.0, 1.0);
        lastPidPower = pidPower;
        lastFfPower = ffPower;

        updateStability(Math.abs(error) <= tuning.toleranceRpm());
    }

    private void updateIntegral(double error, double dt) {
        if (Math.abs(error) <= tuning.integralZoneRpm()) {
            integralAccum = RobotAction.clamp(integralAccum + error * dt,
                    -tuning.integralMaxAccum(), tuning.integralMaxAccum());
        } else {
            integralAccum = 0.0;
        }
    }

    private double feedforwardPower() {
        if (Math.abs(targetRpm) < RobotConstants.SHOOTER_FF_MIN_RPM) {
            return 0.0;
        }
        return tuning.kS() * Math.signum(targetRpm) + tuning.kV() * targetRpm;
    }

    private void updateStability(boolean withinTolerance) {
        if (withinTolerance) {
            if (!previouslyWithinTolerance) {
                withinSinceMs = nowMs;
            }
            if (nowMs - withinSinceMs >= tuning.stabilityMs()) {
                stable = true;
            }
        } else {
            stable = false;
        }
        previouslyWithinTolerance = withinTolerance;
    }

    private void resetController() {
        integralAccum = 0.0;
        previousErrorRpm = 0.0;
        fresh = true;
        stable = false;
        previouslyWithinTolerance = false;
    }

    private ShooterTuning activeTuning() {
        return tuning != null ? tuning : ShooterTuning.DEFAULTS;
    }

    private void readTuning() {
        ShooterTuning next = tuningSupplier.get();
        if (next == null || !next.isValid()) {
            if (!Objects.equals(next, lastRejected) || rejectedTuningCount == 0) {
                rejectEventPending = true;
            }
            lastRejected = next;
            rejectedTuningCount++;
            if (tuning == null) {
                accept(ShooterTuning.DEFAULTS);
            }
            return;
        }
        lastRejected = null;
        if (!next.equals(tuning)) {
            accept(next);
        }
    }

    private void accept(ShooterTuning next) {
        boolean change = tuning != null;
        tuning = next;
        tuningEventPending = true;
        if (change) {
            // Deliberate non-archive addition: an accepted change clears integral and
            // readiness; the archive's tuning was static.
            resetController();
        }
    }

    private void emitEvents(RobotAction.Builder out) {
        boolean pulsing = feeder.isPulsing();
        if (pulsing && !wasPulsing) {
            out.event("shooter.feed.start", nowMs, Collections.emptyMap());
        } else if (!pulsing && wasPulsing) {
            out.event("shooter.feed.end", nowMs, Collections.emptyMap());
        }
        wasPulsing = pulsing;
        if (tuningEventPending) {
            out.event("shooter.tuning", nowMs, tuningData(tuning));
            tuningEventPending = false;
        }
        if (rejectEventPending) {
            out.event("shooter.tuning.rejected", nowMs,
                    Map.of("count", (double) rejectedTuningCount));
            rejectEventPending = false;
        }
    }

    private static Map<String, Double> tuningData(ShooterTuning t) {
        Map<String, Double> data = new LinkedHashMap<>();
        data.put("kS", t.kS());
        data.put("kV", t.kV());
        data.put("kP", t.kP());
        data.put("kI", t.kI());
        data.put("kD", t.kD());
        data.put("integralZoneRpm", t.integralZoneRpm());
        data.put("integralMaxAccum", t.integralMaxAccum());
        data.put("toleranceRpm", t.toleranceRpm());
        data.put("stabilityMs", (double) t.stabilityMs());
        return data;
    }

    public double targetRpm() {
        return targetRpm;
    }

    /** Wheel RPM from shooterRight, or NaN when the sensor is missing. */
    public double measuredRpm() {
        return measuredRpm;
    }

    public boolean isOpenLoop() {
        return openLoop;
    }

    public double appliedPower() {
        return appliedPower;
    }

    public double lastPidPower() {
        return lastPidPower;
    }

    public double lastFeedforwardPower() {
        return lastFfPower;
    }

    public double integralAccum() {
        return integralAccum;
    }

    /** Last accepted tuning snapshot, or null before the first tick. */
    public ShooterTuning tuning() {
        return tuning;
    }

    public int rejectedTuningCount() {
        return rejectedTuningCount;
    }

    public PulseFeeder.Phase feederPhase() {
        return feeder.phase();
    }
}
