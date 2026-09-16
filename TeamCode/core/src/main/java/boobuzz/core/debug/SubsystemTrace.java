package boobuzz.core.debug;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.subsystem.IDrive;
import boobuzz.core.subsystem.IIntake;
import boobuzz.core.subsystem.IShooter;
import boobuzz.core.subsystem.ISubsystem;
import boobuzz.core.subsystem.ITurret;
import boobuzz.core.subsystem.Subsystems;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.List;

/** Records downward subsystem calls without changing the subsystem contracts. */
public final class SubsystemTrace {

    private final List<Call> calls = new ArrayList<>();

    /** Immutable call reference retained until the dispatcher serializes it. */
    public record Call(String sub, String op, List<Object> args) {
        public Call {
            args = args == null ? List.of() : List.copyOf(args);
        }
    }

    public static Subsystems wrap(Subsystems source, SubsystemTrace trace) {
        return new Subsystems(
                new RecordingDrive(source.drive(), trace),
                new RecordingShooter(source.shooter(), trace),
                new RecordingIntake(source.intake(), trace),
                new RecordingTurret(source.turret(), trace));
    }

    public synchronized void call(String subsystem, String operation, Object... args) {
        calls.add(new Call(subsystem, operation, List.of(args)));
    }

    public synchronized List<Call> drainCalls() {
        List<Call> result = List.copyOf(calls);
        calls.clear();
        return result;
    }

    private abstract static class RecordingSubsystem<T extends ISubsystem> implements ISubsystem {
        final T delegate;
        final SubsystemTrace trace;

        RecordingSubsystem(T delegate, SubsystemTrace trace) {
            this.delegate = delegate;
            this.trace = trace;
        }

        @Override public void observe(RobotState state) { delegate.observe(state); }
        @Override public void update(RobotAction.Builder out) { delegate.update(out); }
    }

    private static final class RecordingDrive extends RecordingSubsystem<IDrive> implements IDrive {
        RecordingDrive(IDrive delegate, SubsystemTrace trace) { super(delegate, trace); }

        @Override public void manual(double vx, double vy, double omega) {
            trace.call("drive", "manual", vx, vy, omega);
            delegate.manual(vx, vy, omega);
        }

        @Override public void follow(PathRequest request) {
            trace.call("drive", "follow", request);
            delegate.follow(request);
        }

        @Override public void turnTo(double headingRad) {
            trace.call("drive", "turnTo", headingRad);
            delegate.turnTo(headingRad);
        }

        @Override public void stop() {
            trace.call("drive", "stop");
            delegate.stop();
        }

        @Override public void resetPose(Pose pose) {
            trace.call("drive", "resetPose", pose.x(), pose.y(), pose.heading());
            delegate.resetPose(pose);
        }

        @Override public boolean pathDone() { return delegate.pathDone(); }
        @Override public Pose pose() { return delegate.pose(); }
    }

    private static final class RecordingShooter
            extends RecordingSubsystem<IShooter> implements IShooter {
        RecordingShooter(IShooter delegate, SubsystemTrace trace) { super(delegate, trace); }

        @Override public void spinUp(double rpm) {
            trace.call("shooter", "spinUp", rpm);
            delegate.spinUp(rpm);
        }

        @Override public void spinDown() {
            trace.call("shooter", "spinDown");
            delegate.spinDown();
        }

        @Override public boolean isReady() { return delegate.isReady(); }
        @Override public void feed() {
            trace.call("shooter", "feed");
            delegate.feed();
        }
        @Override public boolean isFeeding() { return delegate.isFeeding(); }
    }

    private static final class RecordingIntake
            extends RecordingSubsystem<IIntake> implements IIntake {
        RecordingIntake(IIntake delegate, SubsystemTrace trace) { super(delegate, trace); }

        @Override public void run(double power) {
            trace.call("intake", "run", power);
            delegate.run(power);
        }

        @Override public void stop() {
            trace.call("intake", "stop");
            delegate.stop();
        }

        @Override public boolean hasBall() { return delegate.hasBall(); }
    }

    private static final class RecordingTurret
            extends RecordingSubsystem<ITurret> implements ITurret {
        RecordingTurret(ITurret delegate, SubsystemTrace trace) { super(delegate, trace); }

        @Override public void aimAt(double fieldX, double fieldY) {
            trace.call("turret", "aimAt", fieldX, fieldY);
            delegate.aimAt(fieldX, fieldY);
        }

        @Override public void scan() {
            trace.call("turret", "scan");
            delegate.scan();
        }

        @Override public void hold() {
            trace.call("turret", "hold");
            delegate.hold();
        }

        @Override public boolean onTarget() { return delegate.onTarget(); }
        @Override public double angleRad() { return delegate.angleRad(); }
    }
}
