package boobuzz.core.contract;

/**
 * Edge-triggered request. It is an event, not a level: "throw" rather than
 * "keep throwing."
 *
 * <p>The lifecycle is tracked by {@link RequestStatus}. There is no command-based
 * scheduler: one {@link Intent} per loop has no resource conflict, and the
 * {@code Intent} itself is the arbitration (architecture documentation, §4).
 */
public record Request(int id, RequestType type, double[] params) {

    public static Request of(int id, RequestType type, double... params) {
        return new Request(id, type, params);
    }

    public double param(int index, double fallback) {
        return (params != null && index < params.length) ? params[index] : fallback;
    }
}
