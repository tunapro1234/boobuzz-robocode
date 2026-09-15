package boobuzz.core.hal;

/**
 * Tek kare gamepad okumasi. docs/protokol.md'deki {@code gamepad} alaninin karsiligi.
 *
 * <p>Simde pygame klavyesi/joystick'i doldurur, robotta OpMode'un {@code gamepad1}'i.
 * :core hangisi oldugunu bilmez (anayasa kural 3).
 *
 * <p>Stick degerleri -1..1, tetikler 0..1. {@code ly} yukari ITERKEN NEGATIFTIR
 * (hem pygame hem FTC boyle verir); ileri yon isteyen {@code -ly} kullanir.
 */
public record GamepadState(double lx, double ly, double rx, double ry,
                           boolean a, boolean b, boolean x, boolean y,
                           boolean lb, boolean rb,
                           double lt, double rt,
                           Dpad dpad) {

    public enum Dpad { NONE, UP, DOWN, LEFT, RIGHT }

    public static GamepadState neutral() {
        return new GamepadState(0, 0, 0, 0,
                false, false, false, false,
                false, false,
                0, 0,
                Dpad.NONE);
    }
}
