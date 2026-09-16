package boobuzz.core.controller.autos;

import boobuzz.core.controller.auto.AutoSequence;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

/** Registry for the six supported autonomous routines. */
public final class AutoRegistry {

    private static final Map<String, Supplier<AutoSequence>> FACTORIES;

    static {
        Map<String, Supplier<AutoSequence>> factories = new LinkedHashMap<>();
        factories.put("BlueDoggy6Piece", BlueDoggy6Piece::build);
        factories.put("RedDoggy6Piece", RedDoggy6Piece::build);
        factories.put("BlueMissionary9Piece", BlueMissionary9Piece::build);
        factories.put("RedMissionary9Piece", RedMissionary9Piece::build);
        factories.put("BlueMissionary9PieceLever", BlueMissionary9PieceLever::build);
        factories.put("RedMissionary9PieceLever", RedMissionary9PieceLever::build);
        FACTORIES = Map.copyOf(factories);
    }

    private AutoRegistry() {}

    public static AutoSequence build(String name) {
        Supplier<AutoSequence> factory = FACTORIES.get(Objects.requireNonNull(name, "name"));
        if (factory == null) {
            throw new IllegalArgumentException("unknown auto: " + name + "; expected " + names());
        }
        return factory.get();
    }

    public static List<String> names() {
        return List.copyOf(FACTORIES.keySet());
    }

    public static Map<String, Supplier<AutoSequence>> factories() {
        return FACTORIES;
    }
}
