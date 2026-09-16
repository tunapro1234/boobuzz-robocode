package boobuzz.core.contract;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** UPWARD feedback. The engine produces it; the controller consumes it. */
public record Feedback(WorldSnapshot world, List<RequestStatus> statuses, long t) {

    public Feedback {
        statuses = Collections.unmodifiableList(new ArrayList<>(statuses));
    }

    public static Feedback of(WorldSnapshot world) {
        return new Feedback(world, java.util.Collections.emptyList(), world.t());
    }
}
