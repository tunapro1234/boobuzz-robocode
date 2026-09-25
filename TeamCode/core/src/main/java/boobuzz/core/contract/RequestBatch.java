package boobuzz.core.contract;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** One controller tick: a level stream plus edge-triggered requests and cancels. */
public record RequestBatch(RequestStream stream, List<Request> requests, int[] cancels) {

    /** Reserved cancellation id used only by RobotLoop during an engine handoff. */
    public static final int CANCEL_ALL = Integer.MIN_VALUE;

    public RequestBatch {
        stream = stream == null ? RequestStream.idle() : stream;
        requests = requests == null ? Collections.emptyList()
                : Collections.unmodifiableList(new ArrayList<>(requests));
        cancels = cancels == null ? new int[0] : cancels.clone();
    }

    public RequestBatch(RequestStream stream, List<Request> requests) {
        this(stream, requests, new int[0]);
    }

    public static RequestBatch idle() {
        return new RequestBatch(RequestStream.idle(), Collections.emptyList(), new int[0]);
    }

    public static RequestBatch of(Request request) {
        return new RequestBatch(RequestStream.idle(), Collections.singletonList(request), new int[0]);
    }

    public static RequestBatch cancelAll() {
        return new RequestBatch(RequestStream.idle(), Collections.emptyList(),
                new int[] {CANCEL_ALL});
    }

    /**
     * The requests an engine should start: a request whose own id is cancelled in the same
     * batch is rejected "cancelled" here and never started (review B08 minor 1).
     */
    public static List<Request> withoutCancelled(RequestBatch batch,
                                                 List<RequestStatus> statuses) {
        List<Request> kept = new ArrayList<>();
        for (Request request : batch.requests) {
            boolean cancelled = false;
            for (int id : batch.cancels) {
                if (id != CANCEL_ALL && id == request.id()) {
                    cancelled = true;
                    break;
                }
            }
            if (cancelled) {
                statuses.add(RequestStatus.rejected(request.id(), "cancelled"));
            } else {
                kept.add(request);
            }
        }
        return kept;
    }

    @Override
    public int[] cancels() {
        return cancels.clone();
    }
}
