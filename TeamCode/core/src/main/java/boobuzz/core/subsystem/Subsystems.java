package boobuzz.core.subsystem;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;

import java.util.Objects;

import boobuzz.core.subsystem.stub.StubTurret;

/** The fixed-order set of robot subsystems. */
public record Subsystems(IDrive drive, IShooter shooter, IIntake intake, ITurret turret) {

    public Subsystems(IDrive drive, IShooter shooter, IIntake intake) {
        this(drive, shooter, intake, new StubTurret());
    }

    public Subsystems {
        Objects.requireNonNull(drive, "drive");
        Objects.requireNonNull(shooter, "shooter");
        Objects.requireNonNull(intake, "intake");
        Objects.requireNonNull(turret, "turret");
    }

    public void observe(RobotState state) {
        drive.observe(state);
        shooter.observe(state);
        intake.observe(state);
        turret.observe(state);
    }

    public RobotAction update() {
        return update(0L);
    }

    /** Builds one action after all subsystems have observed the current sample. */
    public RobotAction update(long tMs) {
        RobotAction.Builder out = new RobotAction.Builder();
        drive.update(out);
        shooter.update(out);
        intake.update(out);
        turret.update(out);
        return out.build();
    }
}
