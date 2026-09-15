package boobuzz.core.logic.engine;

import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;
import boobuzz.core.logic.Subsystem;
import boobuzz.core.logic.drive.DriveSubsystem;
import boobuzz.core.hal.Mechanism;

import java.util.List;

/** C1: ham odometri geri bildirimi ve manuel drive subsystem'i. */
public final class C1DriveEngine implements RobotEngine {

    private final DriveSubsystem drive;
    private final List<Subsystem> subsystems;

    public C1DriveEngine(Mechanism mechanism) {
        drive = DriveSubsystem.manual(mechanism);
        subsystems = List.of(drive);
    }

    @Override
    public String name() {
        return "C1Drive";
    }

    @Override
    public Feedback sense(long now, RobotState state) {
        for (Subsystem subsystem : subsystems) {
            subsystem.observe(now, state);
        }
        WorldSnapshot world = new WorldSnapshot(
                state.t(), state.pinpoint(), state.yaw(), state.voltage());
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

    public String frontLeftMotor() { return drive.frontLeftMotor(); }

    public String frontRightMotor() { return drive.frontRightMotor(); }

    public String backLeftMotor() { return drive.backLeftMotor(); }

    public String backRightMotor() { return drive.backRightMotor(); }

    public Mechanism mechanism() { return drive.mechanism(); }
}
