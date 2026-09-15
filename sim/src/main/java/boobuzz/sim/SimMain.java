package boobuzz.sim;

import boobuzz.core.RobotLoop;
import boobuzz.core.controller.Controller;
import boobuzz.core.contract.Drive;
import boobuzz.core.controller.GamepadController;
import boobuzz.core.contract.Intent;
import boobuzz.core.engine.C1DriveEngine;
import boobuzz.core.engine.RobotEngine;
import boobuzz.core.mechanism.Mechanism;
import boobuzz.core.pedro.PedroDriveEngine;

import com.pedropathing.math.Pose;

import java.nio.file.Path;

/**
 * Headless sim kosucusu.
 *
 * <pre>
 * java -cp ... boobuzz.sim.SimMain --mechanism ../mechanism.yaml --steps 500 --dt 20
 * </pre>
 *
 * <p>RobotLoop'u SimHal + C1DriveEngine + GamepadController ile kosturur,
 * N adim sonra {@code bye} gonderir.
 */
public final class SimMain {

    private SimMain() {}

    public static void main(String[] args) throws Exception {
        Args a = Args.parse(args);

        Mechanism mechanism = Mechanism.load(a.mechanism);
        System.out.printf("mechanism: %s  motorlar=%s%n", a.mechanism, mechanism.motorNames());

        try (SimHal hal = new SimHal(mechanism, a.host, a.port, a.dtMs,
                a.seed, new Pose(a.x, a.y, a.h), a.connectTimeoutMs)) {

            RobotEngine engine;
            if ("pedro".equals(a.engine)) {
                engine = new PedroDriveEngine(mechanism);
                System.out.printf("engine: %s%n", engine.name());
            } else {
                C1DriveEngine c1 = new C1DriveEngine(mechanism);
                engine = c1;
                System.out.printf("engine: %s  fl=%s fr=%s bl=%s br=%s%n",
                        c1.name(), c1.frontLeftMotor(), c1.frontRightMotor(),
                        c1.backLeftMotor(), c1.backRightMotor());
            }

            // Varsayilan: gamepad sunucudan gelir (pygame). --drive headless duman
            // testi icin sabit bir niyet verir; viewer'siz kosuda gamepad hep sifirdir.
            Controller controller;
            if (a.pathId != null) {
                Drive fixed = new Drive.FollowPath(a.pathId);
                controller = fb -> Intent.of(fixed);
                System.out.printf("controller: FollowPath(%s)%n", a.pathId);
            } else if (a.drive == null) {
                controller = new GamepadController(hal);
                System.out.println("controller: gamepad (sunucudan)");
            } else {
                Drive fixed = new Drive.Manual(a.drive[0], a.drive[1], a.drive[2]);
                controller = fb -> Intent.of(fixed);
                System.out.printf("controller: sabit Manual(%.2f, %.2f, %.2f)%n",
                        a.drive[0], a.drive[1], a.drive[2]);
            }

            RobotLoop loop = new RobotLoop(hal, engine, controller);

            long wallStart = System.nanoTime();
            for (int i = 0; i < a.steps; i++) {
                try {
                    loop.tick();
                } catch (ServerClosedException e) {
                    // Viewer penceresi kapandi; bu bir hata degil, kosunun sonu.
                    System.out.println("sunucu kapandi, kosu bitti.");
                    break;
                }
            }
            double wallSec = (System.nanoTime() - wallStart) / 1e9;

            long simMs = hal.now();
            System.out.printf("%d tick bitti.  sim=%.2fs  gercek=%.2fs  hizlanma=%.1fx%n",
                    loop.ticks(), simMs / 1000.0, wallSec,
                    wallSec > 0 ? (simMs / 1000.0) / wallSec : 0.0);
            System.out.printf("son poz: x=%.2f y=%.2f h=%.3f rad%n",
                    hal.read().pinpoint().x(), hal.read().pinpoint().y(),
                    hal.read().pinpoint().heading());
            if (hal.truth() != null) {
                System.out.printf("gercek : x=%.2f y=%.2f h=%.3f rad%n",
                        hal.truth().x(), hal.truth().y(), hal.truth().heading());
            }
        }
    }

    /** Kucuk argüman ayristirici; kutuphane getirmeye degmez. */
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
        String engine = "c1";
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
                    case "--engine" -> a.engine = engine(next(argv, ++i, key));
                    case "--path" -> a.pathId = next(argv, ++i, key);
                    case "--connect-timeout" ->
                            a.connectTimeoutMs = Integer.parseInt(next(argv, ++i, key));
                    default -> throw new IllegalArgumentException("bilinmeyen argüman: " + key);
                }
            }
            if (a.pathId != null && !"pedro".equals(a.engine)) {
                throw new IllegalArgumentException("--path yalnizca --engine pedro ile kullanilir");
            }
            if (a.pathId != null && a.drive != null) {
                throw new IllegalArgumentException("--path ile --drive birlikte kullanilamaz");
            }
            return a;
        }

        private static String engine(String value) {
            if (!"c1".equals(value) && !"pedro".equals(value)) {
                throw new IllegalArgumentException("--engine c1|pedro bekliyor: " + value);
            }
            return value;
        }

        private static double[] triple(String value) {
            String[] parts = value.split(",");
            if (parts.length != 3) {
                throw new IllegalArgumentException("--drive vx,vy,omega bekliyor: " + value);
            }
            return new double[] {
                    Double.parseDouble(parts[0].trim()),
                    Double.parseDouble(parts[1].trim()),
                    Double.parseDouble(parts[2].trim())};
        }

        private static String next(String[] argv, int i, String key) {
            if (i >= argv.length) {
                throw new IllegalArgumentException(key + " bir deger bekliyor");
            }
            return argv[i];
        }
    }
}
