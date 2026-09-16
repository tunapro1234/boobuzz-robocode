package boobuzz.core.subsystem;

import boobuzz.core.contract.PathRequest;

import com.pedropathing.math.Pose;

/** Narrow drive mechanism API consumed by engines. */
public interface IDrive extends ISubsystem {

    void manual(double vx, double vy, double omega);

    void follow(PathRequest request);

    /** Rotate in place to a heading without translating. */
    default void turnTo(double headingRad) {
        Pose current = pose();
        follow(PathRequest.goTo(new Pose(current.x(), current.y(), headingRad),
                PathRequest.Constraints.defaults()));
    }

    void stop();

    boolean pathDone();

    Pose pose();
}
