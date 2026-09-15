package boobuzz.core.logic.engine;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;
import boobuzz.core.logic.Subsystem;
import boobuzz.core.logic.drive.DriveSubsystem;
import boobuzz.core.logic.drive.PathRegistry;
import boobuzz.core.mechanism.Mechanism;

import java.util.List;

/** C1 manuel surusu koruyup GoTo/FollowPath/Hold'u Pedro'ya veren engine. */
public final class PedroDriveEngine implements RobotEngine {

    private final DriveSubsystem drive;
    private final List<Subsystem> subsystems;

    public PedroDriveEngine(Mechanism mechanism) {
        this(mechanism, new PathRegistry());
    }

    public PedroDriveEngine(Mechanism mechanism, PathRegistry paths) {
        drive = DriveSubsystem.pedro(mechanism, paths);
        subsystems = List.of(drive);
    }

    @Override
    public String name() {
        return "PedroDrive";
    }

    @Override
    public Feedback sense(long now, RobotState state) {
        for (Subsystem subsystem : subsystems) {
            subsystem.observe(now, state);
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
