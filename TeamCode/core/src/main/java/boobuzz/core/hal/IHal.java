package boobuzz.core.hal;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.IGamepadSource;

/**
 * L1 - hardware abstraction. This is the ONLY difference between simulator and robot.
 *
 * <p>Two implementations: {@code RealHal} (TeamCode/Android) and {@code SimHal}
 * (:sim, socket to the Python physics server). There is NO {@code if (isSim)}
 * above this interface (constitution rule 3).
 */
public interface IHal extends IGamepadSource {

    /** HAL clock in milliseconds; not wall time so the simulator can run faster. */
    long now();

    /** Read sensors. */
    RobotState read();

    /** Apply motor/servo commands. */
    void write(RobotAction action);
}
