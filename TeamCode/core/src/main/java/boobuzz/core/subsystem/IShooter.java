package boobuzz.core.subsystem;

/** Narrow shooter mechanism API consumed by engines. */
public interface IShooter extends ISubsystem {

    void spinUp(double rpm);

    void spinDown();

    boolean isReady();

    void feed();

    boolean isFeeding();
}
