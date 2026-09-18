package boobuzz.core.subsystem.intake;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.IIntake;

/** Logical signed intake power; hardware direction is configured by the HAL. */
public final class PowerIntake implements IIntake {

    private double power;

    @Override
    public void observe(RobotState state) {
        // Intake feedback is intentionally unsensed in this phase.
    }

    @Override
    public void run(double power) {
        this.power = Double.isFinite(power)
                ? Math.max(-1.0, Math.min(1.0, power)) : 0.0;
    }

    @Override
    public void stop() {
        power = 0.0;
    }

    @Override
    public boolean hasBall() {
        return false;
    }

    @Override
    public void update(RobotAction.Builder out) {
        out.motor(RobotConstants.INTAKE_MOTOR_NAME, power);
    }
}
