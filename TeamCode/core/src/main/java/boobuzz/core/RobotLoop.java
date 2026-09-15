package boobuzz.core;

import boobuzz.core.controller.Controller;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.logic.RobotEngine;
import boobuzz.core.hal.Hal;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;

/**
 * Tick. Bes satir, sirasi sabit (docs/mimari.md §1).
 *
 * <p>Tek thread, deterministik. Tekrarlanmayan sistem ne debug edilir ne egitilir.
 * Geri ok yoktur: Engine, Controller'i cagirmaz - bu yuzden istek durumlari
 * bir tick gecikmelidir.
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

    /** Bir tick kosar. */
    public void tick() {
        long now = hal.now();

        RobotState state = hal.read();
        Feedback feedback = engine.sense(now, state);    // YUKARI
        Intent intent = controller.decide(feedback);     // L3
        RobotAction action = engine.act(intent);         // ASAGI
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
