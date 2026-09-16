package boobuzz.core.logic;

import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.WorldSnapshot;

import java.util.List;

/**
 * L2. Bidirectional: {@link #sense} reads upward, {@link #act} executes downward.
 * Robot and simulator run the same engine implementation through this interface.
 */
public interface IRobotEngine {

    String name();

    /** UP: world view from one raw sensor sample. */
    WorldSnapshot sense(RobotState state);

    /** DOWN: dispatch one request batch to the subsystem set. */
    void act(RequestBatch batch);

    /** Action assembled by the most recent {@link #act} call. */
    default RobotAction action() {
        return RobotAction.zero();
    }

    /** Statuses produced by the preceding act call, consumed on the next tick. */
    default List<RequestStatus> drainStatuses() {
        return List.of();
    }

    /** Compatibility view for callers that still use the old feedback seam. */
    default Feedback sense(long now, RobotState state) {
        return new Feedback(sense(state), drainStatuses(), now);
    }
}
