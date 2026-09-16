package boobuzz.core.subsystem.stub;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.subsystem.IIntake;

import java.util.Map;

/** Timing-free intake stub that records commands and emits state-change events. */
public final class StubIntake implements IIntake {

    private double power;
    private double emittedPower;
    private long nowMs;
    private boolean ballPresent;

    @Override
    public void observe(RobotState state) {
        nowMs = state.t();
    }

    @Override
    public void run(double power) {
        this.power = Double.isFinite(power) ? power : 0.0;
    }

    @Override
    public void stop() {
        power = 0.0;
    }

    @Override
    public boolean hasBall() {
        return ballPresent;
    }

    @Override
    public void update(RobotAction.Builder out) {
        if (Double.doubleToLongBits(power) == Double.doubleToLongBits(emittedPower)) {
            return;
        }
        if (power == 0.0) {
            out.event("intake.off", nowMs, Map.of());
        } else {
            out.event("intake.on", nowMs, Map.of());
        }
        emittedPower = power;
    }

    /** Test hook for the future sensor-backed implementation. */
    void setBallPresent(boolean present) {
        ballPresent = present;
    }
}
