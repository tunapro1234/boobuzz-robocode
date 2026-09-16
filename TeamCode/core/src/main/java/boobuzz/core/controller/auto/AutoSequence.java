package boobuzz.core.controller.auto;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/** Immutable-step autonomous sequence with a mutable execution cursor. */
public final class AutoSequence {

    private final List<AutoStep> steps;
    private int cursor;

    public AutoSequence(List<? extends AutoStep> steps) {
        Objects.requireNonNull(steps, "steps");
        this.steps = Collections.unmodifiableList(new ArrayList<>(steps));
    }

    public List<AutoStep> steps() {
        return steps;
    }

    public int size() {
        return steps.size();
    }

    public int index() {
        return cursor;
    }

    public AutoStep current() {
        return isDone() ? null : steps.get(cursor);
    }

    public boolean isDone() {
        return cursor >= steps.size();
    }

    public void advance() {
        if (!isDone()) {
            cursor++;
        }
    }

    public double progress() {
        return steps.isEmpty() ? 1.0 : (double) cursor / steps.size();
    }

    public String currentName() {
        AutoStep step = current();
        return step == null ? "DONE" : step.name();
    }

    public String name() {
        return currentName();
    }

    /** Starting pose recorded on the first movement step, or the origin for an empty sequence. */
    public Pose startPose() {
        for (AutoStep step : steps) {
            if (step instanceof AutoStep.Path path) {
                return path.startPose();
            }
        }
        return Pose.zero();
    }

    public boolean isFinished() {
        return isDone();
    }

    // Compatibility names used by the archived telemetry code.
    public int getCurrentCommandIndex() {
        return index();
    }

    public int getTotalCommands() {
        return size();
    }

    public String getCurrentCommandName() {
        return currentName();
    }

    public double getProgress() {
        return progress();
    }
}
