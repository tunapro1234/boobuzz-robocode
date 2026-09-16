package boobuzz.core.hal;

/** Gamepad source. SimHal fills it from the socket; RealHal from the OpMode. */
@FunctionalInterface
public interface GamepadSource {
    GamepadState get();
}
