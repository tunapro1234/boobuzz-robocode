package boobuzz.core.contract;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

/** A timestamped subsystem event carried across the HAL seam. */
public record Event(String name, long tMs, Map<String, Double> data) {

    public Event {
        name = Objects.requireNonNull(name, "name");
        data = data == null ? Collections.emptyMap()
                : Collections.unmodifiableMap(new LinkedHashMap<>(data));
    }
}
