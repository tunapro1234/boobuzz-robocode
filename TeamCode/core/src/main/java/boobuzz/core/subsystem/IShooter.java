package boobuzz.core.subsystem;

/** Narrow shooter mechanism API consumed by engines. */
public interface IShooter extends ISubsystem {

    void spinUp(double rpm);

    void spinDown();

    boolean isReady();

    void feed();

    boolean isFeeding();

    /** Command the one hood angle (degrees, clipped to the mechanism range). */
    void setHoodAngleDeg(double angleDeg);

    /** Estimated only: no hood angle sensor exists. */
    boolean hoodSettled();

    /**
     * Operator-held mechanism recovery only: open-loop flywheel power in [-1, 1], closed
     * loop and readiness off. Never drives against a measured rotation: a sign change
     * coasts until {@link #isStopped()}. {@link #spinUp}/{@link #spinDown} leave this mode.
     */
    void runOpenLoop(double power);

    /** Manual feeder power in [-1, 1] for recovery; stops any pulse. 0 stops the feeder. */
    void setFeederPower(double power);

    /** Measured flywheel speed is known and inside the readiness tolerance of zero. */
    boolean isStopped();
}
