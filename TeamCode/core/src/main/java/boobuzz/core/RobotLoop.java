package boobuzz.core;

import boobuzz.core.controller.IController;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.logic.IRobotEngine;
import boobuzz.core.hal.IHal;

import java.util.List;
import java.util.Objects;

/**
 * Tick. Five lines, fixed order (architecture documentation, §1).
 *
 * <p>Single thread, deterministic. A system that cannot be replayed can be neither
 * debugged nor trained. There is no feedback loop: the engine does not call the
 * controller, so request statuses lag by one tick.
 */
public final class RobotLoop {

    private final IHal hal;
    private final List<IRobotEngine> engines;
    private IRobotEngine engine;
    private final IController controller;
    private long ticks;

    public RobotLoop(IHal hal, IRobotEngine engine, IController controller) {
        this(hal, List.of(Objects.requireNonNull(engine, "engine")), engine, controller);
    }

    /** Builds a loop with selectable engines sharing one subsystem set. */
    public RobotLoop(IHal hal, List<? extends IRobotEngine> engines,
                     IRobotEngine initialEngine, IController controller) {
        this.hal = Objects.requireNonNull(hal, "hal");
        this.engines = List.copyOf(engines);
        this.engine = Objects.requireNonNull(initialEngine, "initial engine");
        this.controller = Objects.requireNonNull(controller, "controller");
        if (this.engines.isEmpty() || !this.engines.contains(initialEngine)) {
            throw new IllegalArgumentException("initial engine must be in engine list");
        }
    }

    /** Runs one tick. */
    public void tick() {
        RobotState state = hal.read();
        WorldSnapshot snapshot = engine.sense(state);    // UP
        Feedback feedback = new Feedback(snapshot, engine.drainStatuses(), state.t());
        RequestBatch batch = controller.decide(feedback); // L3
        int switchIndex = switchIndex(batch);
        if (switchIndex >= 0 && switchIndex < engines.size()
                && engines.get(switchIndex) != engine) {
            // Quiesce the old owner now.  The selected engine starts on the next tick.
            engine.act(RequestBatch.cancelAll());
            setEngine(engines.get(switchIndex));
            batch = withoutSwitchRequests(batch);
        } else {
            engine.act(withoutSwitchRequests(batch));     // DOWN
        }
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

    public void setEngine(IRobotEngine engine) {
        this.engine = Objects.requireNonNull(engine, "engine");
    }

    private static int switchIndex(RequestBatch batch) {
        if (batch == null) {
            return -1;
        }
        for (var request : batch.requests()) {
            if (request.type() == boobuzz.core.contract.RequestType.SWITCH_ENGINE) {
                double index = request.param(0, Double.NaN);
                if (Double.isFinite(index) && index == Math.rint(index)) {
                    return (int) index;
                }
            }
        }
        return -1;
    }

    private static RequestBatch withoutSwitchRequests(RequestBatch batch) {
        if (batch == null) {
            return RequestBatch.idle();
        }
        List<boobuzz.core.contract.Request> requests = batch.requests().stream()
                .filter(request -> request.type()
                        != boobuzz.core.contract.RequestType.SWITCH_ENGINE)
                .toList();
        return new RequestBatch(batch.stream(), requests, batch.cancels());
    }
}
