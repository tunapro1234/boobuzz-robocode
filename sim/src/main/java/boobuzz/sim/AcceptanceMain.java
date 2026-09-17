package boobuzz.sim;

import boobuzz.core.RobotFactory;
import boobuzz.core.RobotLoop;
import boobuzz.core.controller.IController;
import boobuzz.core.contract.Event;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.GamepadState;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.IHal;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.RobotConstants;

import com.pedropathing.math.Pose;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Small, direct-HAL acceptance scenario runner for Chapter A.
 *
 * <p>This class deliberately does not use the debug SocketController.  The
 * Python wrapper owns the physics process; this process owns one SimHal client
 * and the normal RobotLoop/RobotFactory path, so the trace exercises the same
 * transport and engine bytecode used by the simulator entry point.
 */
public final class AcceptanceMain {

    private static final int DEFAULT_DT_MS = 20;
    private static final int DRIVE_TICKS = 1000;
    private static final int CANCEL_TICKS = 101;
    private static final Pose START = new Pose(72.0, 72.0, 0.0);
    private static final Pose TARGET = new Pose(120.0, 72.0, 0.0);
    private static final int PATH_ID = 1;

    private AcceptanceMain() {}

    public static void main(String[] args) throws Exception {
        int status;
        try {
            status = run(args);
        } catch (Throwable error) {
            error.printStackTrace(System.err);
            status = 1;
        }
        if (status != 0) {
            System.exit(status);
        }
    }

    /** Runs one fresh-process scenario and returns a shell status. */
    static int run(String[] args) throws Exception {
        Arguments options = Arguments.parse(args);
        ScenarioController controller = new ScenarioController(options.scenario);
        Mechanism mechanism = RobotConstants.mechanism();
        RecordingHal hal = null;
        List<Trace> traces = new ArrayList<>();
        List<RequestStatus> allStatuses = new ArrayList<>();
        List<RobotAction> actions = new ArrayList<>();

        try (SimHal sim = new SimHal(mechanism, options.host, options.port, options.dtMs,
                options.seed, START, options.connectTimeoutMs);
             RobotLoop loop = createLoop(sim, mechanism, options.engine, controller)) {
            hal = controller.hal;
            for (int tick = 0; tick < options.ticks; tick++) {
                loop.tick();
                RobotAction action = hal.lastAction;
                actions.add(action);
                allStatuses.addAll(controller.lastFeedbackStatuses);
                traces.add(Trace.of(options.scenario == Scenario.DRIVE ? "A-drive" : "A-cancel",
                        options.seed, sim.now(),
                        loop.engine().name(), controller.lastFeedbackStatuses,
                        TARGET, sim.read().pinpoint(), sim.truth(), action));
                if (options.scenario == Scenario.CANCEL && tick == CANCEL_TICKS) {
                    throw new IllegalStateException("cancel scenario tick schedule exceeded");
                }
            }
            // One final feedback sample makes a terminal status generated on the
            // last engine tick visible to the acceptance result without another
            // physics step.
            allStatuses.addAll(controller.lastFeedbackStatuses);
            Result result = validate(options, traces, allStatuses, actions, loop);
            for (Trace trace : traces) {
                System.out.println(trace.toJson());
            }
            System.out.println("ACCEPTANCE_RESULT " + result.toJson());
            return result.ok ? 0 : 1;
        } finally {
            // The try-with-resources closes SimHal and RobotLoop.  This branch
            // is intentionally empty: no external process is owned by Java.
        }
    }

    private static RobotLoop createLoop(SimHal sim, Mechanism mechanism, String engine,
                                        ScenarioController controller) {
        controller.hal = new RecordingHal(sim);
        return RobotFactory.createWithController(controller.hal, mechanism, engine, controller);
    }

    private enum Scenario { DRIVE, CANCEL }

    /** Controller schedule is the A03 contract, expressed as one deterministic state machine. */
    private static final class ScenarioController implements IController {
        private final Scenario scenario;
        private int tick;
        private RecordingHal hal;
        private List<RequestStatus> lastFeedbackStatuses = Collections.emptyList();

        private ScenarioController(Scenario scenario) {
            this.scenario = scenario;
        }

        @Override
        public RequestBatch decide(Feedback feedback) {
            lastFeedbackStatuses = feedback == null || feedback.statuses() == null
                    ? Collections.emptyList() : new ArrayList<>(feedback.statuses());
            int current = tick++;
            if (scenario == Scenario.DRIVE) {
                return current == 0
                        ? RequestBatch.of(Request.path(PATH_ID, PathRequest.named("test-line")))
                        : RequestBatch.idle();
            }
            if (current == 0) {
                return RequestBatch.of(Request.path(PATH_ID, PathRequest.named("test-line")));
            }
            if (current >= 20 && current < 40) {
                return new RequestBatch(RequestStream.manual(0.20, 0.0, 0.0),
                        Collections.emptyList(), new int[0]);
            }
            if (current == 40) {
                return RequestBatch.cancelAll();
            }
            if (current == 60) {
                return RequestBatch.of(Request.switchEngine(60, 1));
            }
            if (current == 80) {
                return RequestBatch.of(Request.switchEngine(80, 0));
            }
            return RequestBatch.idle();
        }
    }

    /** Captures immutable actions while preserving the ordinary SimHal boundary. */
    private static final class RecordingHal implements IHal {
        private final SimHal delegate;
        private RobotAction lastAction = RobotAction.zero();

        private RecordingHal(SimHal delegate) {
            this.delegate = delegate;
        }

        @Override public long now() { return delegate.now(); }
        @Override public RobotState read() { return delegate.read(); }
        @Override public GamepadState get() { return delegate.get(); }

        @Override
        public void write(RobotAction action) {
            lastAction = action == null ? RobotAction.zero() : action;
            delegate.write(lastAction);
        }
    }

    private static final class Arguments {
        private Scenario scenario = Scenario.DRIVE;
        private String engine = "direct";
        private String host = SimHal.DEFAULT_HOST;
        private int port = SimHal.DEFAULT_PORT;
        private long seed = 1L;
        private int dtMs = DEFAULT_DT_MS;
        private int ticks = DRIVE_TICKS;
        private int connectTimeoutMs = RobotConstants.SIM_CONNECT_TIMEOUT_MS;

        private static Arguments parse(String[] argv) {
            Arguments result = new Arguments();
            for (int i = 0; i < argv.length; i++) {
                String key = argv[i];
                switch (key) {
                    case "--scenario" -> {
                        String value = next(argv, ++i, key);
                        result.scenario = switch (value) {
                            case "A-drive", "drive" -> Scenario.DRIVE;
                            case "A-cancel", "cancel" -> Scenario.CANCEL;
                            default -> throw new IllegalArgumentException(
                                    "unknown acceptance scenario: " + value);
                        };
                        result.ticks = result.scenario == Scenario.DRIVE
                                ? DRIVE_TICKS : CANCEL_TICKS;
                    }
                    case "--engine" -> result.engine = next(argv, ++i, key);
                    case "--host" -> result.host = next(argv, ++i, key);
                    case "--port" -> result.port = Integer.parseInt(next(argv, ++i, key));
                    case "--seed" -> result.seed = Long.parseLong(next(argv, ++i, key));
                    case "--dt" -> result.dtMs = Integer.parseInt(next(argv, ++i, key));
                    case "--ticks" -> result.ticks = Integer.parseInt(next(argv, ++i, key));
                    case "--connect-timeout" -> result.connectTimeoutMs =
                            Integer.parseInt(next(argv, ++i, key));
                    default -> throw new IllegalArgumentException("unknown option: " + key);
                }
            }
            if (!(result.engine.equals("direct") || result.engine.equals("cplx1")
                    || result.engine.equals("cplx_engine_1"))) {
                throw new IllegalArgumentException("engine must be direct or cplx1");
            }
            if (result.dtMs <= 0 || result.ticks <= 0) {
                throw new IllegalArgumentException("dt and ticks must be positive");
            }
            return result;
        }

        private static String next(String[] argv, int index, String key) {
            if (index >= argv.length) {
                throw new IllegalArgumentException(key + " requires a value");
            }
            return argv[index];
        }
    }

    private static Result validate(Arguments options, List<Trace> traces,
                                   List<RequestStatus> statuses, List<RobotAction> actions,
                                   RobotLoop loop) {
        List<RequestStatus> pathStatuses = statuses.stream()
                .filter(status -> status.id() == PATH_ID).toList();
        long terminalCount = pathStatuses.stream().filter(RequestStatus::terminal).count();
        boolean pathDone = pathStatuses.stream().anyMatch(status ->
                status.state() == RequestStatus.State.DONE);
        Pose finalTruth = traces.isEmpty() ? null : traces.get(traces.size() - 1).truth;
        boolean finite = finalTruth != null && finite(finalTruth);
        boolean ok;
        List<String> failures = new ArrayList<>();
        if (options.scenario == Scenario.DRIVE) {
            ok = pathDone && terminalCount == 1 && finite;
            if (!pathDone) failures.add("test-line did not reach DONE");
            if (terminalCount != 1) failures.add("path terminal count=" + terminalCount);
            if (!finite) failures.add("final truth is missing or non-finite");
            if (finite) {
                double error = distance(finalTruth, TARGET);
                double heading = Math.abs(Math.toDegrees(angleDelta(finalTruth.heading(),
                        TARGET.heading())));
                if (error > 0.5) failures.add(String.format(Locale.US,
                        "truth target error %.4f in", error));
                if (heading > 1.0) failures.add(String.format(Locale.US,
                        "truth heading error %.4f deg", heading));
            }
        } else {
            ok = finite && terminalCount == 1;
            if (!finite) failures.add("final truth is missing or non-finite");
            if (terminalCount != 1) failures.add("path terminal count=" + terminalCount);
            for (int tick : new int[] {40, 60, 80}) {
                if (tick >= actions.size() || !zero(actions.get(tick))) {
                    ok = false;
                    failures.add("non-zero wheels on control tick " + tick);
                }
            }
            if (pathStatuses.stream().anyMatch(status ->
                    status.state() == RequestStatus.State.DONE)) {
                ok = false;
                failures.add("cancelled path unexpectedly reached DONE");
            }
        }
        return new Result(ok, options.scenario.name(), options.engine, options.seed,
                loop.ticks(), finalTruth, failures);
    }

    private static boolean finite(Pose pose) {
        return Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private static double distance(Pose a, Pose b) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y());
    }

    private static double angleDelta(double a, double b) {
        double delta = a - b;
        while (delta > Math.PI) delta -= 2.0 * Math.PI;
        while (delta < -Math.PI) delta += 2.0 * Math.PI;
        return delta;
    }

    private static boolean zero(RobotAction action) {
        return action.motors().values().stream().allMatch(value ->
                Double.isFinite(value) && Math.abs(value) < 1e-9);
    }

    private static final class Trace {
        private final String scenario;
        private final long seed;
        private final long tMs;
        private final String engine;
        private final List<RequestStatus> statuses;
        private final Pose target;
        private final Pose truth;
        private final Pose sensed;
        private final RobotAction action;

        private Trace(String scenario, long seed, long tMs, String engine,
                      List<RequestStatus> statuses, Pose target, Pose truth,
                      Pose sensed, RobotAction action) {
            this.scenario = scenario;
            this.seed = seed;
            this.tMs = tMs;
            this.engine = engine;
            this.statuses = statuses == null ? Collections.emptyList()
                    : List.copyOf(statuses);
            this.target = target;
            this.truth = truth;
            this.sensed = sensed;
            this.action = action == null ? RobotAction.zero() : action;
        }

        static Trace of(String scenario, long seed, long tMs, String engine,
                        List<RequestStatus> statuses, Pose target,
                        Pose sensed, Pose truth, RobotAction action) {
            return new Trace(scenario, seed, tMs, engine, statuses, target,
                    truth, sensed, action);
        }

        String toJson() {
            StringBuilder out = new StringBuilder(1024);
            out.append('{');
            field(out, "scenario", quote(scenario));
            field(out, "seed", Long.toString(seed));
            field(out, "t_ms", Long.toString(tMs));
            field(out, "engine", quote(engine));
            field(out, "request_statuses", statusesJson(statuses));
            field(out, "commanded_target", poseJson(target));
            field(out, "truth", poseJson(truth));
            field(out, "sensed_pose", poseJson(sensed));
            field(out, "motors", numberMapJson(action.motors()));
            field(out, "servos", numberMapJson(action.servos()));
            field(out, "events", eventsJson(action.events()));
            out.append('}');
            return out.toString();
        }
    }

    private static final class Result {
        private final boolean ok;
        private final String scenario;
        private final String engine;
        private final long seed;
        private final long ticks;
        private final Pose finalTruth;
        private final List<String> failures;

        private Result(boolean ok, String scenario, String engine, long seed, long ticks,
                       Pose finalTruth, List<String> failures) {
            this.ok = ok;
            this.scenario = scenario;
            this.engine = engine;
            this.seed = seed;
            this.ticks = ticks;
            this.finalTruth = finalTruth;
            this.failures = List.copyOf(failures);
        }

        String toJson() {
            StringBuilder out = new StringBuilder(256);
            out.append('{');
            field(out, "ok", Boolean.toString(ok));
            field(out, "scenario", quote(scenario));
            field(out, "engine", quote(engine));
            field(out, "seed", Long.toString(seed));
            field(out, "ticks", Long.toString(ticks));
            field(out, "final_truth", poseJson(finalTruth));
            field(out, "failures", stringsJson(failures));
            out.append('}');
            return out.toString();
        }
    }

    private static void field(StringBuilder out, String name, String value) {
        if (out.length() > 1) out.append(',');
        out.append(quote(name)).append(':').append(value);
    }

    private static String quote(String value) {
        return "\"" + Json.escape(value == null ? "" : value) + "\"";
    }

    private static String poseJson(Pose pose) {
        if (pose == null) return "null";
        return "{\"x\":" + number(pose.x()) + ",\"y\":" + number(pose.y())
                + ",\"h\":" + number(pose.heading()) + '}';
    }

    private static String numberMapJson(Map<String, Double> values) {
        StringBuilder out = new StringBuilder("{");
        boolean first = true;
        for (Map.Entry<String, Double> entry : values.entrySet()) {
            if (!first) out.append(',');
            first = false;
            out.append(quote(entry.getKey())).append(':').append(number(entry.getValue()));
        }
        return out.append('}').toString();
    }

    private static String statusesJson(List<RequestStatus> statuses) {
        StringBuilder out = new StringBuilder("[");
        for (int i = 0; i < statuses.size(); i++) {
            if (i > 0) out.append(',');
            RequestStatus status = statuses.get(i);
            out.append("{\"id\":").append(status.id())
                    .append(",\"state\":").append(quote(status.state().name()))
                    .append(",\"progress\":").append(number(status.progress()))
                    .append(",\"note\":").append(quote(status.note())).append('}');
        }
        return out.append(']').toString();
    }

    private static String eventsJson(List<Event> events) {
        StringBuilder out = new StringBuilder("[");
        for (int i = 0; i < events.size(); i++) {
            if (i > 0) out.append(',');
            Event event = events.get(i);
            out.append("{\"name\":").append(quote(event.name()))
                    .append(",\"t_ms\":").append(event.tMs())
                    .append(",\"data\":").append(numberMapJson(event.data())).append('}');
        }
        return out.append(']').toString();
    }

    private static String stringsJson(List<String> values) {
        StringBuilder out = new StringBuilder("[");
        for (int i = 0; i < values.size(); i++) {
            if (i > 0) out.append(',');
            out.append(quote(values.get(i)));
        }
        return out.append(']').toString();
    }

    private static String number(double value) {
        if (!Double.isFinite(value)) {
            throw new IllegalStateException("non-finite acceptance trace value: " + value);
        }
        return Double.toString(value);
    }
}
