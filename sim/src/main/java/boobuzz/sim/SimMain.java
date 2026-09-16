package boobuzz.sim;

import boobuzz.core.RobotLoop;
import boobuzz.core.RobotFactory;
import boobuzz.core.contract.Drive;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.MechanismLoader;

import com.pedropathing.math.Pose;

import java.nio.file.Path;

/**
 * Headless simulator runner.
 *
 * <pre>
 * java -cp ... boobuzz.sim.SimMain --mechanism ../mechanism.yaml --steps 500 --dt 20
 * </pre>
 *
 * <p>Runs RobotLoop with SimHal + CplxEngine1 + GamepadController and sends
 * {@code bye} after N steps.
 */
public final class SimMain {

    private SimMain() {}

    public static void main(String[] args) throws Exception {
        Args a = Args.parse(args);

        Mechanism mechanism = MechanismLoader.load(a.mechanism);
        System.out.printf("mechanism: %s  motors=%s%n", a.mechanism, mechanism.motorNames());

        try (SimHal hal = new SimHal(mechanism, a.host, a.port, a.dtMs,
                a.seed, new Pose(a.x, a.y, a.h), a.connectTimeoutMs)) {

            // Default: the gamepad comes from the server (pygame). --drive provides a
            // fixed intent for a headless smoke test; without a viewer, gamepad stays neutral.
            Drive fixedDrive = null;
            if (a.pathId != null) {
                fixedDrive = new Drive.FollowPath(a.pathId);
                System.out.printf("controller: FollowPath(%s)%n", a.pathId);
            } else if (a.drive == null) {
                System.out.println("controller: gamepad (from server)");
            } else {
                fixedDrive = new Drive.Manual(a.drive[0], a.drive[1], a.drive[2]);
                System.out.printf("controller: fixed Manual(%.2f, %.2f, %.2f)%n",
                        a.drive[0], a.drive[1], a.drive[2]);
            }

            RobotLoop loop = RobotFactory.create(hal, mechanism, fixedDrive);
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
        }
    }

    /** Small argument parser; not worth adding a library. */
    private static final class Args {
        Path mechanism = Path.of("mechanism.yaml");
        String host = SimHal.DEFAULT_HOST;
        int port = SimHal.DEFAULT_PORT;
        int steps = 500;
        int dtMs = 20;
        long seed = 0;
        double x = 0, y = 0, h = 0;
        int connectTimeoutMs = 5000;
        double[] drive = null;
        String pathId = null;

        static Args parse(String[] argv) {
            Args a = new Args();
            for (int i = 0; i < argv.length; i++) {
                String key = argv[i];
                switch (key) {
                    case "--mechanism" -> a.mechanism = Path.of(next(argv, ++i, key));
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
                    case "--connect-timeout" ->
                            a.connectTimeoutMs = Integer.parseInt(next(argv, ++i, key));
                    default -> throw new IllegalArgumentException("unknown argument: " + key);
                }
            }
            if (a.pathId != null && a.drive != null) {
                throw new IllegalArgumentException("--path and --drive cannot be used together");
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
