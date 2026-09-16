package boobuzz.core.subsystem;

/** Narrow intake mechanism API consumed by engines. */
public interface Intake extends Subsystem {

    void run(double power);

    void stop();

    boolean hasBall();
}
