package boobuzz.core;

import boobuzz.core.controller.Controller;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.logic.RobotEngine;
import boobuzz.core.hal.Hal;

/**
 * Tick. Five lines, fixed order (architecture documentation, §1).
 *
 * <p>Single thread, deterministic. A system that cannot be replayed can be neither
 * debugged nor trained. There is no feedback loop: the engine does not call the
 * controller, so request statuses lag by one tick.
 */
public final class RobotLoop {

    private final Hal hal;
    private final RobotEngine engine;
    private final Controller controller;
    private long ticks;

    public RobotLoop(Hal hal, RobotEngine engine, Controller controller) {
        this.hal = hal;
        this.engine = engine;
        this.controller = controller;
    }

    /** Runs one tick. */
    public void tick() {
        long now = hal.now();

        RobotState state = hal.read();
        Feedback feedback = engine.sense(now, state);    // UP
        Intent intent = controller.decide(feedback);     // L3
        RobotAction action = engine.act(intent);         // DOWN
        hal.write(action);

        ticks++;
    }

    public long ticks() {
        return ticks;
    }

    public RobotEngine engine() {
        return engine;
    }
}
