package boobuzz.sim;

/** Protokol ihlali. Sessiz kayma yerine aninda cokme (docs/protokol.md). */
public class SimProtocolException extends RuntimeException {

    public SimProtocolException(String message) {
        super(message);
    }

    public SimProtocolException(String message, Throwable cause) {
        super(message, cause);
    }
}
