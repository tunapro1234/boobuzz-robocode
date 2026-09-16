package boobuzz.core.subsystem;

/** Narrow intake mechanism API consumed by engines. */
public interface IIntake extends ISubsystem {

    void run(double power);

    void stop();

    boolean hasBall();
}
