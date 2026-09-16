package boobuzz.sim;

/**
 * The server closed the connection. This is an expected end, not a protocol
 * violation: Python exits cleanly when the viewer window closes. {@code SimMain}
 * reports it on one line and exits normally; a stack trace would be misleading.
 *
 * <p>It still extends {@link SimProtocolException}, because for callers outside
 * SimMain (tests), lockstep ended prematurely.
 */
public final class ServerClosedException extends SimProtocolException {

    public ServerClosedException(String message) {
        super(message);
    }
}
