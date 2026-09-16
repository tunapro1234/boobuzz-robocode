package boobuzz.core.controller;

import boobuzz.core.contract.GamepadState;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * Controller-only button edge helper.  It compares two frames and never knows
 * which HAL produced them.
 *
 * <p>The mutable cursor makes a controller cheap to use in a tick loop:
 * {@link #update(GamepadState, long)} advances one frame, while {@link #toggle}
 * retains only the requested toggle state.
 */
public final class Buttons {

    private static final String[] NAMES = {
            "a", "b", "x", "y", "lb", "rb", "back", "start",
            "left_bumper", "right_bumper"
    };

    private GamepadState previous;
    private GamepadState current;
    private long previousTimeMs;
    private long currentTimeMs;
    private final Map<String, Boolean> toggleStates = new HashMap<>();
    private final Map<String, Long> heldSinceMs = new HashMap<>();
    private final Set<String> toggledThisFrame = new HashSet<>();

    public Buttons(GamepadState previous, GamepadState current) {
        this(previous, current, 0L, 0L);
    }

    public Buttons(GamepadState previous, GamepadState current,
                   long previousTimeMs, long currentTimeMs) {
        this.previous = Objects.requireNonNull(previous, "previous gamepad");
        this.current = Objects.requireNonNull(current, "current gamepad");
        this.previousTimeMs = previousTimeMs;
        this.currentTimeMs = currentTimeMs;
        for (String name : NAMES) {
            if (held(name)) {
                heldSinceMs.put(name, currentTimeMs);
            }
        }
    }

    public Buttons(GamepadState current) {
        this(GamepadState.neutral(), current, 0L, 0L);
    }

    /** Advances the helper to the next HAL-clocked frame. */
    public void update(GamepadState next, long nowMs) {
        previous = current;
        previousTimeMs = currentTimeMs;
        current = Objects.requireNonNull(next, "next gamepad");
        currentTimeMs = nowMs;
        toggledThisFrame.clear();
        for (String name : NAMES) {
            if (held(name)) {
                heldSinceMs.putIfAbsent(name, nowMs);
            } else {
                heldSinceMs.remove(name);
            }
        }
    }

    public GamepadState previous() {
        return previous;
    }

    public GamepadState current() {
        return current;
    }

    public boolean pressed(String name) {
        return value(previous, name) == false && value(current, name);
    }

    public boolean released(String name) {
        return value(previous, name) && !value(current, name);
    }

    public boolean held(String name) {
        return value(current, name);
    }

    /** Flips the named state once per rising edge and returns its new value. */
    public boolean toggle(String name) {
        String key = normalize(name);
        boolean state = toggleStates.getOrDefault(key, false);
        if (pressed(key) && toggledThisFrame.add(key)) {
            state = !state;
            toggleStates.put(key, state);
        }
        return state;
    }

    public boolean heldFor(String name, double seconds) {
        if (!Double.isFinite(seconds) || seconds < 0.0 || !held(name)) {
            return false;
        }
        Long since = heldSinceMs.get(normalize(name));
        return since != null && currentTimeMs - since >= Math.round(seconds * 1000.0);
    }

    public long currentTimeMs() {
        return currentTimeMs;
    }

    private static boolean value(GamepadState state, String name) {
        return switch (normalize(name)) {
            case "a" -> state.a();
            case "b" -> state.b();
            case "x" -> state.x();
            case "y" -> state.y();
            case "lb" -> state.lb();
            case "rb" -> state.rb();
            case "back" -> state.back();
            case "start" -> state.start();
            default -> throw new IllegalArgumentException("unknown button: " + name);
        };
    }

    private static String normalize(String name) {
        Objects.requireNonNull(name, "button name");
        return switch (name.toLowerCase(java.util.Locale.ROOT)) {
            case "left_bumper" -> "lb";
            case "right_bumper" -> "rb";
            default -> name.toLowerCase(java.util.Locale.ROOT);
        };
    }
}
