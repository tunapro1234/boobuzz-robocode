package boobuzz.core.subsystem;

/** Narrow shooter mechanism API consumed by engines. */
public interface Shooter extends Subsystem {

    void spinUp(double rpm);

    void spinDown();

    boolean isReady();

    void feed();

    boolean isFeeding();
}
