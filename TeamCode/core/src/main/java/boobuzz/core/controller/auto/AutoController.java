package boobuzz.core.controller.auto;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import java.util.Objects;

/** Autonomous controller facade over the shared sequence runner. */
public final class AutoController implements boobuzz.core.controller.IController {

    private final SequenceRunner runner;

    public AutoController(AutoSequence sequence) {
        runner = new SequenceRunner(Objects.requireNonNull(sequence, "sequence"));
    }

    @Override
    public RequestBatch decide(Feedback feedback) {
        return runner.decide(feedback);
    }

    public AutoSequence sequence() {
        return runner.sequence();
    }

    public boolean isDone() {
        return runner.isDone();
    }

    public boolean isFinished() {
        return runner.isFinished();
    }

    public boolean isFailed() {
        return runner.isFailed();
    }

    public RequestStatus failure() {
        return runner.failure();
    }

    public String failureNote() {
        return runner.failureNote();
    }
}
