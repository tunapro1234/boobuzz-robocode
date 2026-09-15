package boobuzz.core.controller;

import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.hal.GamepadSource;
import boobuzz.core.hal.GamepadState;

import com.pedropathing.math.Pose;

/**
 * L3 - surucu girdisini niyete cevirir.
 *
 * <p>Esleme {@code SmokeTeleop} ile ayni: sol stick surus, sag stick X donus,
 * stick'ler ters isaretli (yukari itmek negatif deger verir).
 *
 * <p>Olu bolge (deadband) burada, HAL'de degil: gamepad donanimi merkeze tam
 * donmez ve bu her iki tarafta da (sim, robot) ayni problemdir.
 *
 * <p><b>Field-oriented:</b> varsayilan ACIK. Sol stick saha cercevesinde okunur
 * (yukari = saha {@code +x}), heading ile geri dondurulup robot cercevesine
 * cevrilir. {@link Drive} tipi ve Engine bundan haberdar degil - donusum tamamen
 * L3'te biter, asagi giden yine robot cerceveli {@code Drive.Manual}'dir.
 *
 * <p>{@code b} modu degistirir, {@code y} heading'i sifirlar. Sifirlama HAL'e
 * dokunmaz; burada tutulan bir offset'tir (anayasa kural 3: :core donanimin
 * hangisi oldugunu bilmez, sensoru de yeniden ayarlamaz).
 */
public final class GamepadController implements Controller {

    private static final double DEADBAND = 0.05;

    private final GamepadSource gamepads;
    private final double driveScale;

    private boolean fieldOriented = true;
    private double headingOffset = 0.0;
    private boolean prevToggle;
    private boolean prevReset;

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

        // Kenar tetikleme: tus basili tutuldugunda her tick'te tetiklenmesin.
        if (g.b() && !prevToggle) {
            fieldOriented = !fieldOriented;
        }
        prevToggle = g.b();
        if (g.y() && !prevReset) {
            headingOffset = rawHeading(feedback);
        }
        prevReset = g.y();

        double forward = deadband(-g.ly()) * driveScale;
        double left = deadband(-g.lx()) * driveScale;
        double omega = deadband(-g.rx()) * driveScale; // CCW, her iki modda da robot cerceveli

        double vx = forward;
        double vy = left;
        if (fieldOriented) {
            // Stick saha niyeti; robot cercevesine -heading donusu ile gecilir.
            double h = rawHeading(feedback) - headingOffset;
            double cos = Math.cos(h);
            double sin = Math.sin(h);
            vx = forward * cos + left * sin;
            vy = -forward * sin + left * cos;
        }
        return Intent.of(new Drive.Manual(vx, vy, omega));
    }

    /** C1'de localizer pinpoint'in kendisidir; poz yoksa heading bilinmiyor demektir. */
    private static double rawHeading(Feedback feedback) {
        if (feedback == null || feedback.world() == null) {
            return 0.0;
        }
        Pose pose = feedback.world().pose();
        return pose == null ? feedback.world().yaw() : pose.heading();
    }

    private static double deadband(double value) {
        if (Math.abs(value) < DEADBAND) {
            return 0.0;
        }
        // Olu bolgeden sonra yeniden olceklendir; esikte sicrama olmasin.
        double sign = Math.signum(value);
        return sign * (Math.abs(value) - DEADBAND) / (1.0 - DEADBAND);
    }

    /** Telemetri ve testler icin. */
    public boolean fieldOriented() { return fieldOriented; }

    public double headingOffset() { return headingOffset; }
}
