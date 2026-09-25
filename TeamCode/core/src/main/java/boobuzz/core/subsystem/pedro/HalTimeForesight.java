package boobuzz.core.subsystem.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;

import java.util.Objects;
import java.util.function.LongSupplier;

/**
 * Pedro Foresight whose hold timeout runs on HAL time instead of the wall clock.
 *
 * <p>Pedro core 3.0.0 {@code Foresight} keeps a private {@code utils.Timer}
 * (backed by {@code System.nanoTime}). Verified with {@code javap -c -p}:
 * <ul>
 *   <li>{@code reset()} resets the timer and sets {@code resetTimer = true};</li>
 *   <li>the first {@code calculateHold(...)} after that resets the timer again and
 *       sets {@code resetTimer = false};</li>
 *   <li>{@code timeoutCondition()} is {@code !resetTimer && timer ms > timeoutConstraint};</li>
 *   <li>{@code calculateHold} clears {@code busy} when {@code busy && timeoutCondition()}.</li>
 * </ul>
 * The timer is read nowhere else. In a simulator that runs faster or slower than
 * real time, that wall-clock read made {@code isBusy()} (and so path DONE) land on
 * a load-dependent tick. This subclass mirrors the same arm/reset points and
 * evaluates the same comparison against HAL milliseconds, keeping Pedro's
 * {@code timeoutConstraint} value (library default 100.0 ms) untouched.
 */
final class HalTimeForesight extends Foresight {

    private final LongSupplier halTimeMs;
    /** Mirrors Pedro's private {@code !resetTimer}; false after construction and reset. */
    private boolean holdTimerArmed;
    private long holdTimerStartMs;
    /** True for an in-place turn: only heading/translation/velocity may end it. */
    private boolean settleOnly;

    HalTimeForesight(ForesightConfig config, LongSupplier halTimeMs) {
        super(config);
        this.halTimeMs = Objects.requireNonNull(halTimeMs, "halTimeMs");
    }

    @Override
    public void reset() {
        super.reset();
        holdTimerArmed = false;
        settleOnly = false;
    }

    /**
     * Resets for an in-place turn hold that ends only when Pedro's heading,
     * translational and velocity constraints hold together, never on the timeout.
     *
     * <p>Pedro's {@code timeoutConstraint} is the archive's path-end timeout
     * (Pedro 2.0.4 {@code PathConstraints(0.99, 100, 1, 1)}): it counts from reaching
     * the parametric end of a travelled path. A hold started from rest arms it on its
     * first tick, so a turn would report done 100 ms after it began whatever its
     * heading error. The archive had no turn time limit: its {@code turnTo} was a
     * busy {@code followPath} and has no caller. The next {@link #reset()} (any
     * {@code follow} or hold start) restores the timeout.
     */
    void resetForSettle() {
        reset();
        settleOnly = true;
    }

    @Override
    public DrivePowers calculateHold(Drivetrain drivetrain, Pose pose, MotionState state,
                                     boolean useHoldScaling, double deltaTimeSeconds) {
        if (!holdTimerArmed) {
            holdTimerStartMs = halTimeMs.getAsLong();
            holdTimerArmed = true;
        }
        return super.calculateHold(drivetrain, pose, state, useHoldScaling, deltaTimeSeconds);
    }

    @Override
    public boolean timeoutCondition() {
        return !settleOnly && holdTimerArmed
                && halTimeMs.getAsLong() - holdTimerStartMs > config.timeoutConstraint.get();
    }
}
