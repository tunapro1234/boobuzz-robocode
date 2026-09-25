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
}
