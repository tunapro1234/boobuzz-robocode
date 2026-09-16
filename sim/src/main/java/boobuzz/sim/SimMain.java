package boobuzz.sim;

import boobuzz.core.RobotLoop;
import boobuzz.core.RobotFactory;
import boobuzz.core.controller.auto.AutoController;
import boobuzz.core.controller.auto.AutoSequence;
import boobuzz.core.controller.IController;
import boobuzz.core.controller.opmodes.AutoRegistry;
import boobuzz.core.contract.Feedback;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.hal.RobotConstants;

import com.pedropathing.math.Pose;

/**
 * Headless simulator runner.
 *
 * <pre>
 * java -cp ... boobuzz.sim.SimMain --steps 500 --dt 20
 * </pre>
 *
 * <p>Runs RobotLoop with SimHal + CplxEngine1 + TeleopController and sends
 * {@code bye} after N steps.
 */
public final class SimMain {

    private SimMain() {}

    public static void main(String[] args) throws Exception {
        int status = run(args);
        if (status != 0) {
            System.exit(status);
        }
    }

    /** Runs one simulation and returns the process status without terminating the JVM. */
    static int run(String[] args) throws Exception {
        Args a = Args.parse(args);

        Mechanism mechanism = RobotConstants.mechanism();
        System.out.printf("mechanism: RobotConstants  motors=%s%n", mechanism.motorNames());
        AutoSequence sequence = a.auto == null ? null : AutoRegistry.build(a.auto);
        AutoController autoController = sequence == null ? null : new AutoController(sequence);
        Pose startPose = sequence == null ? new Pose(a.x, a.y, a.h) : sequence.startPose();
        if (sequence != null) {
            System.out.printf("auto: %s  steps=%d  start=(%.2f, %.2f, %.3f)%n",
                    a.auto, sequence.size(), startPose.x(), startPose.y(), startPose.heading());
        }

        try (SimHal hal = new SimHal(mechanism, a.host, a.port, a.dtMs,
                a.seed, startPose, a.connectTimeoutMs)) {

            // Default: the gamepad comes from the server (pygame). --drive provides a
            // fixed intent for a headless smoke test; without a viewer, gamepad stays neutral.
            RequestStream fixedStream = null;
            IController fixedController = null;
            if (autoController != null) {
                System.out.printf("controller: auto (%s)%n", a.auto);
            } else if (a.pathId != null) {
                fixedController = new FixedPathController(a.pathId);
                System.out.printf("controller: FollowPath(%s)%n", a.pathId);
            } else if (a.drive == null) {
                System.out.println("controller: gamepad (from server)");
            } else {
                fixedStream = RequestStream.manual(a.drive[0], a.drive[1], a.drive[2]);
                System.out.printf("controller: fixed Manual(%.2f, %.2f, %.2f)%n",
                        a.drive[0], a.drive[1], a.drive[2]);
            }

            RobotLoop loop;
            if (autoController != null) {
                loop = RobotFactory.createWithController(
                        hal, mechanism, a.engine, autoController, a.tapPort);
            } else if (fixedController != null) {
                loop = RobotFactory.createWithController(
                        hal, mechanism, a.engine, fixedController, a.tapPort);
            } else if (fixedStream != null) {
                loop = RobotFactory.create(hal, mechanism, a.engine, fixedStream, a.tapPort);
            } else {
                loop = RobotFactory.createWithController(
                        hal, mechanism, a.engine, new boobuzz.core.controller.teleop.TeleopController(hal),
                        a.tapPort);
            }
            System.out.printf("engine: %s%n", loop.engine().name());

            long wallStart = System.nanoTime();
            for (int i = 0; i < a.steps; i++) {
                try {
                    loop.tick();
                } catch (ServerClosedException e) {
                    // The viewer window closed; this is the end of the run, not an error.
                    System.out.println("server closed, run finished.");
                    break;
                }
                if (autoController != null && autoController.isFinished()) {
                    break;
                }
            }
            double wallSec = (System.nanoTime() - wallStart) / 1e9;

            long simMs = hal.now();
            System.out.printf("%d ticks finished.  sim=%.2fs  wall=%.2fs  speedup=%.1fx%n",
                    loop.ticks(), simMs / 1000.0, wallSec,
                    wallSec > 0 ? (simMs / 1000.0) / wallSec : 0.0);
            Pose pose = hal.read().pinpoint();
            System.out.printf("final pose: x=%.2f y=%.2f h=%.3f rad%n",
                    pose.x(), pose.y(), pose.heading());
            Pose truth = hal.truth();
            if (truth != null) {
                System.out.printf("truth : x=%.2f y=%.2f h=%.3f rad%n",
                        truth.x(), truth.y(), truth.heading());
            }
            if (autoController != null) {
                if (autoController.isDone()) {
                    System.out.printf("auto finished: %d/%d steps%n",
                            sequence.size(), sequence.size());
                } else {
                    System.out.printf("auto unfinished: %d/%d steps; %s%n",
                            sequence.index(), sequence.size(), autoController.failureNote());
                    loop.close();
                    return 1;
                }
            }
            loop.close();
        }
        return 0;
    }

    /** Emits a named path request once; requests are edge-triggered. */
    private static final class FixedPathController implements IController {
        private final String pathId;
        private boolean sent;

        private FixedPathController(String pathId) {
            this.pathId = pathId;
        }

        @Override
        public RequestBatch decide(Feedback feedback) {
            if (sent) {
                return RequestBatch.idle();
            }
            sent = true;
            return RequestBatch.of(Request.path(1, PathRequest.named(pathId)));
        }
    }

    /** Small argument parser; not worth adding a library. */
    private static final class Args {
        String host = SimHal.DEFAULT_HOST;
        int port = SimHal.DEFAULT_PORT;
        int steps = 500;
        int dtMs = 20;
        long seed = 0;
        double x = 0, y = 0, h = 0;
        int connectTimeoutMs = 5000;
        double[] drive = null;
        String pathId = null;
        String auto = null;
        String engine = "cplx1";
        int tapPort = RobotConstants.DEBUG_TAP_PORT;

        static Args parse(String[] argv) {
            Args a = new Args();
            for (int i = 0; i < argv.length; i++) {
                String key = argv[i];
                switch (key) {
                    case "--host" -> a.host = next(argv, ++i, key);
                    case "--port" -> a.port = Integer.parseInt(next(argv, ++i, key));
                    case "--steps" -> a.steps = Integer.parseInt(next(argv, ++i, key));
                    case "--dt" -> a.dtMs = Integer.parseInt(next(argv, ++i, key));
                    case "--seed" -> a.seed = Long.parseLong(next(argv, ++i, key));
                    case "--x" -> a.x = Double.parseDouble(next(argv, ++i, key));
                    case "--y" -> a.y = Double.parseDouble(next(argv, ++i, key));
                    case "--h" -> a.h = Double.parseDouble(next(argv, ++i, key));
                    case "--drive" -> a.drive = triple(next(argv, ++i, key));
                    case "--path" -> a.pathId = next(argv, ++i, key);
                    case "--auto" -> a.auto = next(argv, ++i, key);
                    case "--engine" -> a.engine = next(argv, ++i, key);
                    case "--tap-port" -> a.tapPort = Integer.parseInt(next(argv, ++i, key));
                    case "--connect-timeout" ->
                            a.connectTimeoutMs = Integer.parseInt(next(argv, ++i, key));
                    default -> throw new IllegalArgumentException("unknown argument: " + key);
                }
            }
            int driveModes = (a.pathId == null ? 0 : 1)
                    + (a.drive == null ? 0 : 1)
                    + (a.auto == null ? 0 : 1);
            if (driveModes > 1) {
                throw new IllegalArgumentException(
                        "--auto, --path and --drive cannot be used together");
            }
            return a;
        }

        private static double[] triple(String value) {
            String[] parts = value.split(",");
            if (parts.length != 3) {
                throw new IllegalArgumentException("--drive expects vx,vy,omega: " + value);
            }
            return new double[] {
                    Double.parseDouble(parts[0].trim()),
                    Double.parseDouble(parts[1].trim()),
                    Double.parseDouble(parts[2].trim())};
        }

        private static String next(String[] argv, int i, String key) {
            if (i >= argv.length) {
                throw new IllegalArgumentException(key + " expects a value");
            }
            return argv[i];
        }
    }
}
