package boobuzz.core.debug;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;

import java.util.List;
import java.util.Objects;

/** Immutable references captured at one RobotLoop tick; serialized off-thread. */
public record DebugFrame(RobotState state, RobotAction action,
                         List<SubsystemTrace.Call> calls,
                         Feedback feedback, RequestBatch batch) {

    public DebugFrame {
        state = Objects.requireNonNull(state, "state");
        action = Objects.requireNonNull(action, "action");
        calls = calls == null ? List.of() : List.copyOf(calls);
        feedback = Objects.requireNonNull(feedback, "feedback");
        batch = batch == null ? RequestBatch.idle() : batch;
    }
}
