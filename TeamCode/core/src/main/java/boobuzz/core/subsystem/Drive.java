package boobuzz.core.subsystem;

import boobuzz.core.contract.PathRequest;

import com.pedropathing.math.Pose;

/** Narrow drive mechanism API consumed by engines. */
public interface Drive extends Subsystem {

    void manual(double vx, double vy, double omega);

    void follow(PathRequest request);

    void stop();

    boolean pathDone();

    Pose pose();
}
