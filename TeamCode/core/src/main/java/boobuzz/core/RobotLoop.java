package boobuzz.core;

import boobuzz.core.controller.IController;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.debug.DebugTap;
import boobuzz.core.debug.DebugFrame;
import boobuzz.core.debug.SubsystemTrace;
import boobuzz.core.logic.IRobotEngine;
import boobuzz.core.hal.IHal;

import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Objects;
import java.io.File;

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
    private long tickNanosTotal;
    private long maxTickNanos;
    private List<boobuzz.core.contract.RequestStatus> pendingLoopStatuses =
            Collections.emptyList();

    public RobotLoop(IHal hal, IRobotEngine engine, IController controller) {
        this(hal, Collections.singletonList(Objects.requireNonNull(engine, "engine")),
                engine, controller);
    }

    public RobotLoop(IHal hal, IRobotEngine engine, IController controller,
                     int debugTapPort) {
        this(hal, Collections.singletonList(Objects.requireNonNull(engine, "engine")), engine,
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
        this.engines = Collections.unmodifiableList(new ArrayList<>(engines));
        this.engine = Objects.requireNonNull(initialEngine, "initial engine");
        this.controller = Objects.requireNonNull(controller, "controller");
        if (this.engines.isEmpty() || !this.engines.contains(initialEngine)) {
            throw new IllegalArgumentException("initial engine must be in engine list");
        }
        if (debugTapPort != 0) {
            openDebugTap(debugTapPort);
        }
    }

    /** Runs one tick. */
    public void tick() {
        long tickStart = System.nanoTime();
        try {
            RobotState state = hal.read();
            WorldSnapshot snapshot = engine.sense(state);    // UP
            List<boobuzz.core.contract.RequestStatus> feedbackStatuses = new ArrayList<>(
                    engine.drainStatuses());
            feedbackStatuses.addAll(pendingLoopStatuses);
            pendingLoopStatuses = Collections.emptyList();
            Feedback feedback = new Feedback(snapshot, feedbackStatuses, state.t());
            RequestBatch controllerBatch = controller.decide(feedback); // L3
            RequestBatch batch = controllerBatch;
            List<boobuzz.core.contract.RequestStatus> switchStatuses = new ArrayList<>();
            int switchIndex = switchIndex(controllerBatch, engines.size(), switchStatuses);
            pendingLoopStatuses = switchStatuses;
            boolean switched = switchIndex >= 0 && switchIndex < engines.size()
                    && engines.get(switchIndex) != engine;
            if (switched) {
                // Quiesce the old owner now.  The selected engine starts on the next tick.
                engine.act(RequestBatch.cancelAll());
                List<boobuzz.core.contract.RequestStatus> oldStatuses = engine.drainStatuses();
                if (!oldStatuses.isEmpty()) {
                    List<boobuzz.core.contract.RequestStatus> combined = new ArrayList<>(
                            pendingLoopStatuses);
                    combined.addAll(oldStatuses);
                    pendingLoopStatuses = Collections.unmodifiableList(new ArrayList<>(combined));
                }
                setEngine(engines.get(switchIndex));
                batch = withoutSwitchRequests(batch);
            } else {
                engine.act(withoutSwitchRequests(batch));     // DOWN
            }
            // Retained engine instances may still hold their pre-handoff command.
            // Never write that stale action on the switch tick.
            RobotAction action = switched ? RobotAction.zero() : engine.action();
            hal.write(action);                               // HAL

            publishSeams(state, action, feedback, controllerBatch);

            ticks++;
        } finally {
            long elapsed = System.nanoTime() - tickStart;
            tickNanosTotal += elapsed;
            maxTickNanos = Math.max(maxTickNanos, elapsed);
        }
    }

    public long ticks() {
        return ticks;
    }

    public double meanTickMillis() {
        return ticks == 0 ? 0.0 : tickNanosTotal / (ticks * 1_000_000.0);
    }

    public double maxTickMillis() {
        return maxTickNanos / 1_000_000.0;
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

    /** Configures a JSONL bag; opening and writing happen on the tap dispatcher. */
    public boolean openBag(File path, String controllerName) {
        return openBag(path, controllerName, null);
    }

    /** Configures a bag and preserves the sequence start pose for replay. */
    public boolean openBag(File path, String controllerName,
                           com.pedropathing.math.Pose startPose) {
        if (path == null) {
            return true;
        }
        try {
            File absolute = path.getAbsoluteFile();
            File parent = absolute.getParentFile();
            if (absolute.isDirectory()) {
                return false;
            }
            if (parent != null && !parent.exists()
                    && !parent.mkdirs() && !parent.isDirectory()) {
                return false;
            }
            if (!absolute.exists() && !absolute.createNewFile()) {
                return false;
            }
            if (!absolute.canWrite()) {
                return false;
            }
            if (debugTap == null) {
                debugTap = new DebugTap(0);
            }
            debugTap.configureBag(absolute, engine.name(),
                    controllerName == null ? controller.getClass().getSimpleName() : controllerName,
                    boobuzz.core.hal.RobotConstants.constantsHash(), startPose);
            return true;
        } catch (java.io.IOException | RuntimeException e) {
            return false;
        }
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
        if (debugTap == null) {
            if (subsystemTrace != null) subsystemTrace.drainCalls();
            return;
        }
        List<SubsystemTrace.Call> calls = subsystemTrace == null
                ? Collections.emptyList() : subsystemTrace.drainCalls();
        // The loop only hands immutable record references to the dispatcher.
        debugTap.offer(new DebugFrame(state, action, calls, feedback, batch));
    }

    private static int switchIndex(RequestBatch batch, int engineCount,
                                   List<boobuzz.core.contract.RequestStatus> statuses) {
        if (batch == null) {
            return -1;
        }
        int selected = -1;
        for (var request : batch.requests()) {
            if (request.type() == boobuzz.core.contract.RequestType.SWITCH_ENGINE) {
                double index = request.param(0, Double.NaN);
                if (!Double.isFinite(index) || index != Math.rint(index)) {
                    statuses.add(boobuzz.core.contract.RequestStatus.rejected(
                            request.id(), "SWITCH_ENGINE requires an integral index"));
                    continue;
                }
                int target = (int) index;
                if (target < 0 || target >= engineCount) {
                    statuses.add(boobuzz.core.contract.RequestStatus.rejected(
                            request.id(), "SWITCH_ENGINE index is out of range"));
                    continue;
                }
                if (selected < 0) {
                    selected = target;
                    statuses.add(boobuzz.core.contract.RequestStatus.done(request.id()));
                } else {
                    statuses.add(boobuzz.core.contract.RequestStatus.rejected(
                            request.id(), "multiple SWITCH_ENGINE requests in one tick"));
                }
            }
        }
        return selected;
    }

    private static RequestBatch withoutSwitchRequests(RequestBatch batch) {
        if (batch == null) {
            return RequestBatch.idle();
        }
        List<boobuzz.core.contract.Request> requests = new ArrayList<>();
        for (boobuzz.core.contract.Request request : batch.requests()) {
            if (request.type() != boobuzz.core.contract.RequestType.SWITCH_ENGINE) {
                requests.add(request);
            }
        }
        return new RequestBatch(batch.stream(), requests, batch.cancels());
    }
}
