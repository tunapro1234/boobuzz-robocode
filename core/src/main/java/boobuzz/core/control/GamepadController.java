package boobuzz.core.control;

import boobuzz.core.hal.GamepadSource;
import boobuzz.core.hal.GamepadState;

/**
 * L3 - surucu girdisini niyete cevirir.
 *
 * <p>Esleme {@code SmokeTeleop} ile ayni: sol stick surus, sag stick X donus,
 * stick'ler ters isaretli (yukari itmek negatif deger verir).
 *
 * <p>Olu bolge (deadband) burada, HAL'de degil: gamepad donanimi merkeze tam
 * donmez ve bu her iki tarafta da (sim, robot) ayni problemdir.
 */
public final class GamepadController implements Controller {

    private static final double DEADBAND = 0.05;

    private final GamepadSource gamepads;
    private final double driveScale;

    public GamepadController(GamepadSource gamepads) {
        this(gamepads, 1.0);
    }

    public GamepadController(GamepadSource gamepads, double driveScale) {
        this.gamepads = gamepads;
        this.driveScale = driveScale;
    }

    @Override
    public Intent decide(Feedback feedback) {
        GamepadState g = gamepads.get();
        if (g == null) {
            return Intent.idle();
        }
        double vx = deadband(-g.ly()) * driveScale;   // ileri
        double vy = deadband(-g.lx()) * driveScale;   // sol
        double omega = deadband(-g.rx()) * driveScale; // CCW
        return Intent.of(new Drive.Manual(vx, vy, omega));
    }

    private static double deadband(double value) {
        if (Math.abs(value) < DEADBAND) {
            return 0.0;
        }
        // Olu bolgeden sonra yeniden olceklendir; esikte sicrama olmasin.
        double sign = Math.signum(value);
        return sign * (Math.abs(value) - DEADBAND) / (1.0 - DEADBAND);
    }
}
