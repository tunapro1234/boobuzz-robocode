package boobuzz.core.logic.cplx_engine_1;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;
import boobuzz.core.logic.RobotEngine;
import boobuzz.core.logic.Subsystem;
import boobuzz.core.hal.Mechanism;

import java.util.List;

/** Manuel surus ile Pedro GoTo/FollowPath/Hold komutlarini calistiran tek engine. */
public final class CplxEngine1 implements RobotEngine {

    private final DriveSubsystem drive;
    private final List<Subsystem> subsystems;

    public CplxEngine1(Mechanism mechanism) {
        drive = new DriveSubsystem(mechanism, new PathRegistry());
        subsystems = List.of(drive);
    }

    @Override
    public String name() {
        return "cplx_engine_1";
    }

    @Override
    public Feedback sense(long now, RobotState state) {
        for (Subsystem subsystem : subsystems) {
            subsystem.observe(state);
        }
        WorldSnapshot world = new WorldSnapshot(
                state.t(), drive.pose(), state.yaw(), state.voltage());
        return new Feedback(world, drive.drainStatuses(), now);
    }

    @Override
    public RobotAction act(Intent intent) {
        RobotAction.Builder out = new RobotAction.Builder();
        for (Subsystem subsystem : subsystems) {
            subsystem.update(intent, out);
        }
        return out.build();
    }
}
