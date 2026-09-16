package boobuzz.core.logic.cplx_engine_1;

import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Intent;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.logic.RobotEngine;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.pedro.PedroDrive;

import java.util.ArrayList;
import java.util.List;

/** Engine that combines the Pedro drive with the shared subsystem set. */
public final class CplxEngine1 implements RobotEngine {

    private final Subsystems subsystems;
    private List<RequestStatus> pendingStatuses = List.of();
    private RobotAction lastAction = RobotAction.zero();

    public CplxEngine1(Subsystems subsystems) {
        this.subsystems = subsystems;
    }

    @Override
    public String name() {
        return "cplx_engine_1";
    }

    @Override
    public WorldSnapshot sense(RobotState state) {
        subsystems.observe(state);
        return new WorldSnapshot(
                state.t(), subsystems.drive().pose(), state.yaw(), state.voltage());
    }

    @Override
    public void act(Intent intent) {
        applyDrive(intent.drive());
        rejectUnsupportedRequests(intent.newRequests());
        lastAction = subsystems.update();
    }

    public Subsystems subsystems() {
        return subsystems;
    }

    @Override
    public RobotAction action() {
        return lastAction;
    }

    @Override
    public List<RequestStatus> drainStatuses() {
        List<RequestStatus> statuses = pendingStatuses;
        pendingStatuses = List.of();
        return statuses;
    }

    private void applyDrive(Drive command) {
        if (command instanceof Drive.Manual manual) {
            subsystems.drive().manual(manual.vx(), manual.vy(), manual.omega());
        } else if (command instanceof Drive.GoTo goTo) {
            subsystems.drive().follow(PathRequest.goTo(goTo.target(), goTo.constraints()));
        } else if (command instanceof Drive.FollowPath path) {
            subsystems.drive().follow(PathRequest.named(path.pathId()));
        } else {
            subsystems.drive().stop();
        }
    }

    private void rejectUnsupportedRequests(List<Request> requests) {
        if (requests.isEmpty()) {
            return;
        }
        List<RequestStatus> statuses = new ArrayList<>(requests.size());
        for (Request request : requests) {
            statuses.add(RequestStatus.rejected(
                    request.id(), "cplx_engine_1 has no subsystem for this request"));
        }
        pendingStatuses = List.copyOf(statuses);
    }
}
