package boobuzz.core.hal;

/** Gamepad kaynagi. SimHal soketten, RealHal OpMode'dan doldurur. */
@FunctionalInterface
public interface GamepadSource {
    GamepadState get();
}
