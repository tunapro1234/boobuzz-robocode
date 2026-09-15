package boobuzz.core.control;

import java.util.List;

/** YUKARI giden geri besleme. Engine uretir, Controller tuketir. */
public record Feedback(WorldSnapshot world, List<RequestStatus> statuses, long t) {

    public Feedback {
        statuses = List.copyOf(statuses);
    }

    public static Feedback of(WorldSnapshot world) {
        return new Feedback(world, List.of(), world.t());
    }
}
