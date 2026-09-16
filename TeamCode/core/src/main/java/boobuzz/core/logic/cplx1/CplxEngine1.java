package boobuzz.core.logic.cplx1;

import boobuzz.core.contract.Intent;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.logic.direct.DirectEngine;
import boobuzz.core.logic.IRobotEngine;
import boobuzz.core.subsystem.Subsystems;

import java.util.List;

/** Engine that combines the Pedro drive with the shared subsystem set. */
public final class CplxEngine1 implements IRobotEngine {

    private final Subsystems subsystems;
    private final DirectEngine requestEngine;

    public CplxEngine1(Subsystems subsystems) {
        this.subsystems = subsystems;
        this.requestEngine = new DirectEngine(subsystems);
    }

    @Override
    public String name() {
        return "cplx1";
    }

    @Override
    public WorldSnapshot sense(RobotState state) {
        subsystems.observe(state);
        return new WorldSnapshot(
                state.t(), subsystems.drive().pose(), state.yaw(), state.voltage());
    }

    @Override
    public void act(Intent intent) {
        requestEngine.act(intent);
    }

    public Subsystems subsystems() {
        return subsystems;
    }

    @Override
    public RobotAction action() {
        return requestEngine.action();
    }

    @Override
    public List<RequestStatus> drainStatuses() {
        return requestEngine.drainStatuses();
    }
}
