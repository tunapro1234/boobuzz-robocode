package boobuzz.core.logic;

import boobuzz.core.logic.cplx1.CplxEngine1;
import boobuzz.core.logic.direct.DirectEngine;
import boobuzz.core.subsystem.Subsystems;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/** Stable engine selectors shared by factory construction and runtime handoff. */
public final class EngineRegistry {

    public static final int DIRECT_INDEX = 0;
    public static final int CPLX1_INDEX = 1;
    public static final int CPLX2_INDEX = 2;
    public static final int VISION_A_INDEX = 3;
    public static final int VISION_B_INDEX = 4;
    public static final int RANGE_INDEX = 5;

    public static final String DIRECT_NAME = "direct";
    public static final String CPLX1_NAME = "cplx1";
    public static final String CPLX1_ALIAS = "cplx_engine_1";

    private EngineRegistry() {}

    /** Creates the implemented engines over one shared subsystem set. */
    public static List<IRobotEngine> create(Subsystems subsystems) {
        return create(subsystems, MechanismProfile.STUB);
    }

    /** Both engines share one subsystem set and one mechanism profile. */
    public static List<IRobotEngine> create(Subsystems subsystems, MechanismProfile profile) {
        Objects.requireNonNull(subsystems, "subsystems");
        Objects.requireNonNull(profile, "profile");
        return List.of(new DirectEngine(subsystems, profile), new CplxEngine1(subsystems, profile));
    }

    /** Returns the stable index for a factory/replay selector. */
    public static int indexForName(String selector) {
        Objects.requireNonNull(selector, "engine selector");
        return switch (selector) {
            case DIRECT_NAME -> DIRECT_INDEX;
            case CPLX1_NAME, CPLX1_ALIAS -> CPLX1_INDEX;
            default -> throw new IllegalArgumentException("unknown engine: " + selector
                    + " (expected " + DIRECT_NAME + " or " + CPLX1_NAME + ")");
        };
    }

    /** Returns the canonical bag/factory name for a selector. */
    public static String canonicalName(String selector) {
        return nameForIndex(indexForName(selector));
    }

    /** Returns the canonical name for an implemented selector index. */
    public static String nameForIndex(int index) {
        return switch (index) {
            case DIRECT_INDEX -> DIRECT_NAME;
            case CPLX1_INDEX -> CPLX1_NAME;
            default -> throw new IllegalArgumentException(
                    "engine index is reserved or unknown: " + index);
        };
    }

    /** Returns whether the index has an implementation in this release. */
    public static boolean isImplementedIndex(int index) {
        return index == DIRECT_INDEX || index == CPLX1_INDEX;
    }

    /** Binds implemented engines by their stable names, never by list position. */
    public static Map<Integer, IRobotEngine> bind(List<? extends IRobotEngine> engines) {
        Objects.requireNonNull(engines, "engines");
        Map<Integer, IRobotEngine> bound = new LinkedHashMap<>();
        for (IRobotEngine engine : engines) {
            Objects.requireNonNull(engine, "engine");
            int index = indexForName(engine.name());
            IRobotEngine previous = bound.put(index, engine);
            if (previous != null && previous != engine) {
                throw new IllegalArgumentException("duplicate engine selector: " + engine.name());
            }
        }
        return Collections.unmodifiableMap(new LinkedHashMap<>(bound));
    }

    /** Finds an engine by stable index, or returns null when it is not bound. */
    public static IRobotEngine find(Map<Integer, ? extends IRobotEngine> engines, int index) {
        Objects.requireNonNull(engines, "engines");
        if (!isImplementedIndex(index)) {
            return null;
        }
        return engines.get(index);
    }
}
