package boobuzz.core;

import boobuzz.core.controller.Controller;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.logic.engine.RobotEngine;
import boobuzz.core.hal.Hal;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;
import boobuzz.core.probe.NullProbe;
import boobuzz.core.probe.Probe;

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
    private final Probe probe;

    private long ticks;

    public RobotLoop(Hal hal, RobotEngine engine, Controller controller) {
        this(hal, engine, controller, NullProbe.INSTANCE);
    }

    public RobotLoop(Hal hal, RobotEngine engine, Controller controller, Probe probe) {
        this.hal = hal;
        this.engine = engine;
        this.controller = controller;
        this.probe = probe;
    }

    /** Bir tick kosar. */
    public void tick() {
        long now = hal.now();

        RobotState state = hal.read();
        Feedback feedback = engine.sense(now, state);    // YUKARI
        Intent intent = controller.decide(feedback);     // L3
        RobotAction action = engine.act(intent);         // ASAGI
        hal.write(action);

        probe.publish("state", state, now);
        probe.publish("feedback", feedback, now);
        probe.publish("intent", intent, now);
        probe.publish("action", action, now);
        ticks++;
    }

    public long ticks() {
        return ticks;
    }

    public RobotEngine engine() {
        return engine;
    }
}
