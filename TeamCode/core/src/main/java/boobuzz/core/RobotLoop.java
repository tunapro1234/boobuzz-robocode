package boobuzz.core;

import boobuzz.core.controller.IController;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.logic.IRobotEngine;
import boobuzz.core.hal.IHal;

/**
 * Tick. Five lines, fixed order (architecture documentation, §1).
 *
 * <p>Single thread, deterministic. A system that cannot be replayed can be neither
 * debugged nor trained. There is no feedback loop: the engine does not call the
 * controller, so request statuses lag by one tick.
 */
public final class RobotLoop {

    private final IHal hal;
    private final IRobotEngine engine;
    private final IController controller;
    private long ticks;

    public RobotLoop(IHal hal, IRobotEngine engine, IController controller) {
        this.hal = hal;
        this.engine = engine;
        this.controller = controller;
    }

    /** Runs one tick. */
    public void tick() {
        RobotState state = hal.read();
        WorldSnapshot snapshot = engine.sense(state);    // UP
        Feedback feedback = new Feedback(snapshot, engine.drainStatuses(), state.t());
        Intent intent = controller.decide(feedback);     // L3
        engine.act(intent);                              // DOWN
        RobotAction action = engine.action();
        hal.write(action);                               // HAL

        ticks++;
    }

    public long ticks() {
        return ticks;
    }

    public IRobotEngine engine() {
        return engine;
    }
}
