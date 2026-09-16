package boobuzz.core.subsystem;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;

/** Common subsystem lifecycle: observe one sensor sample, then write one action. */
public interface Subsystem {

    void observe(RobotState state);

    void update(RobotAction.Builder out);
}
