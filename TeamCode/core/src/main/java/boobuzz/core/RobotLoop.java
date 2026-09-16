package boobuzz.core;

import boobuzz.core.controller.IController;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.debug.DebugTap;
import boobuzz.core.debug.SeamJson;
import boobuzz.core.debug.SubsystemTrace;
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
public final class RobotLoop implements AutoCloseable {

    private final IHal hal;
    private final List<IRobotEngine> engines;
    private IRobotEngine engine;
    private final IController controller;
    private DebugTap debugTap;
    private SubsystemTrace subsystemTrace;
    private long ticks;

    public RobotLoop(IHal hal, IRobotEngine engine, IController controller) {
        this(hal, List.of(Objects.requireNonNull(engine, "engine")), engine, controller);
    }

    public RobotLoop(IHal hal, IRobotEngine engine, IController controller,
                     int debugTapPort) {
        this(hal, List.of(Objects.requireNonNull(engine, "engine")), engine,
                controller, debugTapPort);
    }

    /** Builds a loop with selectable engines sharing one subsystem set. */
    public RobotLoop(IHal hal, List<? extends IRobotEngine> engines,
                     IRobotEngine initialEngine, IController controller) {
        this(hal, engines, initialEngine, controller, 0);
    }

    /** Builds a loop and starts a debug tap when {@code debugTapPort != 0}. */
    public RobotLoop(IHal hal, List<? extends IRobotEngine> engines,
                     IRobotEngine initialEngine, IController controller,
                     int debugTapPort) {
        this.hal = Objects.requireNonNull(hal, "hal");
        this.engines = List.copyOf(engines);
        this.engine = Objects.requireNonNull(initialEngine, "initial engine");
        this.controller = Objects.requireNonNull(controller, "controller");
        if (this.engines.isEmpty() || !this.engines.contains(initialEngine)) {
            throw new IllegalArgumentException("initial engine must be in engine list");
        }
        openDebugTap(debugTapPort);
    }

    /** Runs one tick. */
    public void tick() {
        RobotState state = hal.read();
        WorldSnapshot snapshot = engine.sense(state);    // UP
        Feedback feedback = new Feedback(snapshot, engine.drainStatuses(), state.t());
        RequestBatch controllerBatch = controller.decide(feedback); // L3
        RequestBatch batch = controllerBatch;
        int switchIndex = switchIndex(controllerBatch);
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

        publishSeams(state, action, feedback, controllerBatch);

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

    /** Starts or replaces the tap; port zero disables it. */
    public boolean openDebugTap(int port) {
        closeDebugTap();
        if (port == 0) {
            return true;
        }
        try {
            debugTap = new DebugTap(port);
            return true;
        } catch (java.io.IOException e) {
            debugTap = null;
            return false;
        }
    }

    public DebugTap debugTap() {
        return debugTap;
    }

    public void setSubsystemTrace(SubsystemTrace trace) {
        subsystemTrace = trace;
    }

    @Override
    public void close() {
        closeDebugTap();
    }

    private void closeDebugTap() {
        if (debugTap != null) {
            debugTap.close();
            debugTap = null;
        }
    }

    private void publishSeams(RobotState state, RobotAction action,
                              Feedback feedback, RequestBatch batch) {
        if (debugTap == null || !debugTap.enabled()) {
            if (subsystemTrace != null) subsystemTrace.drainCalls();
            return;
        }
        List<java.util.Map<String, Object>> calls = subsystemTrace == null
                ? List.of() : subsystemTrace.drainCalls();
        debugTap.publish(SeamJson.hal(state.t(), state, action));
        debugTap.publish(SeamJson.subsystem(state.t(), calls, action.events()));
        debugTap.publish(SeamJson.logic(state.t(), feedback, batch));
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
