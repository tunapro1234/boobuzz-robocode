package boobuzz.core.contract;

import java.util.List;

/** One controller tick: a level stream plus edge-triggered requests and cancels. */
public record RequestBatch(RequestStream stream, List<Request> requests, int[] cancels) {

    /** Reserved cancellation id used only by RobotLoop during an engine handoff. */
    public static final int CANCEL_ALL = Integer.MIN_VALUE;

    public RequestBatch {
        stream = stream == null ? RequestStream.idle() : stream;
        requests = requests == null ? List.of() : List.copyOf(requests);
        cancels = cancels == null ? new int[0] : cancels.clone();
    }

    public RequestBatch(RequestStream stream, List<Request> requests) {
        this(stream, requests, new int[0]);
    }

    public static RequestBatch idle() {
        return new RequestBatch(RequestStream.idle(), List.of(), new int[0]);
    }

    public static RequestBatch of(Request request) {
        return new RequestBatch(RequestStream.idle(), List.of(request), new int[0]);
    }

    public static RequestBatch cancelAll() {
        return new RequestBatch(RequestStream.idle(), List.of(), new int[] {CANCEL_ALL});
    }

    @Override
    public int[] cancels() {
        return cancels.clone();
    }
}
