package boobuzz.core.contract;

/**
 * Request status. Returned UPWARD to the controller.
 *
 * <p>ONE TICK DELAYED: the engine runs after the controller (constitution rule 4,
 * no feedback loop). The controller sees this tick's request result on the next tick.
 */
public record RequestStatus(int id, State state, double progress, String note) {

    public enum State { ACCEPTED, ACTIVE, DONE, FAILED, REJECTED }

    public static RequestStatus rejected(int id, String note) {
        return new RequestStatus(id, State.REJECTED, 0.0, note);
    }

    public static RequestStatus done(int id) {
        return new RequestStatus(id, State.DONE, 1.0, "");
    }

    public boolean terminal() {
        return state == State.DONE || state == State.FAILED || state == State.REJECTED;
    }
}
