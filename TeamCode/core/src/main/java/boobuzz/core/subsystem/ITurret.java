package boobuzz.core.subsystem;

/** Narrow turret mechanism API; aiming is owned by the complex logic layer. */
public interface ITurret extends ISubsystem {

    void aimAt(double fieldX, double fieldY);

    void scan();

    void hold();

    boolean onTarget();

    double angleRad();
}
