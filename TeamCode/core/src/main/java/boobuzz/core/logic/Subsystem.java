package boobuzz.core.logic;

import boobuzz.core.contract.Intent;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;

/**
 * Bidirectional subsystem boundary within L2.
 *
 * <p>{@link #observe} receives raw state flowing upward, while {@link #update}
 * receives intent flowing downward. Since {@link RobotAction} is immutable,
 * subsystems write onto the same ordered {@link RobotAction.Builder}; the engine
 * materializes the final action once.
 */
public interface Subsystem {

    void observe(RobotState state);

    void update(Intent intent, RobotAction.Builder out);
}
