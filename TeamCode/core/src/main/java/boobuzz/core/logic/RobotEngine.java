package boobuzz.core.logic;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;

/**
 * L2. Bidirectional: {@link #sense} reads upward, {@link #act} executes downward.
 * Robot and simulator run the same engine implementation through this interface.
 */
public interface RobotEngine {

    String name();

    /** UP: world view and request statuses from raw state. */
    Feedback sense(long now, RobotState state);

    /** DOWN: motor/servo command from intent. */
    RobotAction act(Intent intent);
}
