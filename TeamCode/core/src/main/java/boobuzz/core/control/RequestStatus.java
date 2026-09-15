package boobuzz.core.control;

/**
 * Istek durumu. Controller'a YUKARI doner.
 *
 * <p>BIR TICK GECIKMELIDIR: Engine, Controller'dan sonra kosar (anayasa kural 4,
 * geri ok yok). Controller bu tick verdigi istegin sonucunu gelecek tick gorur.
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
