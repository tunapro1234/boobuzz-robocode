package boobuzz.core.subsystem.feeder;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.ISubsystem;

/**
 * Time-based power feeder ported from the archive FeederPowerSubsystem.
 *
 * <p>A pulse runs at {@link RobotConstants#FEEDER_PULSE_POWER} until at least
 * {@link RobotConstants#FEEDER_PULSE_MS} have elapsed. With
 * {@link #requestPulseAndDelay(long)} the motor then stops for the requested
 * post-pulse delay before the next pulse. Time comes only from the observed
 * HAL clock ({@link RobotState#t()}), so a tick sees the phase boundary at the
 * first sample at or after it.
 */
public final class PulseFeeder implements ISubsystem {

    /** Pulse state machine phase. */
    public enum Phase { IDLE, PULSING, GAP }

    private Phase phase = Phase.IDLE;
    private long nowMs;
    private long phaseStartMs;

    private boolean pulseRequested;
    private boolean pulseAndDelayEnabled;
    private long postPulseDelayMs;

    private double power;

    @Override
    public void observe(RobotState state) {
        nowMs = state.t();
    }

    /** Request pulses back to back; a held request restarts a pulse without stopping. */
    public void requestPulse() {
        pulseRequested = true;
    }

    /**
     * Request pulses separated by {@code delayMs} of stopped motor. Repeated calls while
     * held coalesce into one continuous request. The parameter is the POST-PULSE delay,
     * never the pulse duration.
     */
    public void requestPulseAndDelay(long delayMs) {
        if (delayMs < 0) {
            throw new IllegalArgumentException("post-pulse delay must be >= 0 ms: " + delayMs);
        }
        pulseAndDelayEnabled = true;
        postPulseDelayMs = delayMs;
    }

    /** Stop starting new pulses; a pulse already running finishes normally. */
    public void clearRequest() {
        pulseRequested = false;
        pulseAndDelayEnabled = false;
    }

    /** Cancel the pulse or delay immediately and stop the motor. */
    public void stop() {
        power = 0.0;
        phase = Phase.IDLE;
        pulseRequested = false;
        pulseAndDelayEnabled = false;
    }

    /**
     * Manual power, as the archive jam-clear/burst paths use it. Like the archive this
     * does not change the pulse state; callers stop the feeder first when they need that.
     */
    public void setPower(double power) {
        this.power = Double.isFinite(power) ? Math.max(-1.0, Math.min(1.0, power)) : 0.0;
    }

    public Phase phase() {
        return phase;
    }

    public boolean isPulsing() {
        return phase == Phase.PULSING;
    }

    public boolean isInDelay() {
        return phase == Phase.GAP;
    }

    public long delayRemainingMs() {
        return phase == Phase.GAP
                ? Math.max(0L, postPulseDelayMs - (nowMs - phaseStartMs)) : 0L;
    }

    public boolean isRunning() {
        return Math.abs(power) > 0.01;
    }

    @Override
    public void update(RobotAction.Builder out) {
        advance();
        out.motor(RobotConstants.FEEDER_MOTOR_NAME, power);
    }

    private void advance() {
        switch (phase) {
            case GAP:
                if (nowMs - phaseStartMs >= postPulseDelayMs) {
                    phase = Phase.IDLE;
                    if (pulseAndDelayEnabled) {
                        startPulse();
                    }
                }
                return;
            case PULSING:
                if (nowMs - phaseStartMs >= RobotConstants.FEEDER_PULSE_MS) {
                    if (pulseAndDelayEnabled) {
                        phase = Phase.GAP;
                        phaseStartMs = nowMs;
                        power = 0.0;
                    } else if (pulseRequested) {
                        phaseStartMs = nowMs;
                    } else {
                        phase = Phase.IDLE;
                        power = 0.0;
                    }
                }
                return;
            case IDLE:
            default:
                if (pulseRequested || pulseAndDelayEnabled) {
                    startPulse();
                }
        }
    }

    private void startPulse() {
        power = RobotConstants.FEEDER_PULSE_POWER;
        phaseStartMs = nowMs;
        phase = Phase.PULSING;
    }
}
