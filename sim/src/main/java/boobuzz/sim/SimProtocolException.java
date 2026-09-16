package boobuzz.sim;

/** Protocol violation. Fail immediately instead of silently drifting (protocol documentation). */
public class SimProtocolException extends RuntimeException {

    public SimProtocolException(String message) {
        super(message);
    }

    public SimProtocolException(String message, Throwable cause) {
        super(message, cause);
    }
}
