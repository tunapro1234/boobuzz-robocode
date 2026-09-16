package boobuzz.core.contract;

import java.util.List;

/** UPWARD feedback. The engine produces it; the controller consumes it. */
public record Feedback(WorldSnapshot world, List<RequestStatus> statuses, long t) {

    public Feedback {
        statuses = List.copyOf(statuses);
    }

    public static Feedback of(WorldSnapshot world) {
        return new Feedback(world, List.of(), world.t());
    }
}
