package boobuzz.sim;

/**
 * Sunucu baglantiyi kapatti. Protokol ihlali degil, beklenen bir son: viewer
 * penceresi kapaninca Python temiz cikar. {@code SimMain} bunu tek satirla
 * bildirip normal sonlanir; stack trace yaniltici olur.
 *
 * <p>Yine de {@link SimProtocolException} soyundandir, cunku SimMain disindaki
 * cagiranlar (testler) icin lockstep yarida kalmis demektir.
 */
public final class ServerClosedException extends SimProtocolException {

    public ServerClosedException(String message) {
        super(message);
    }
}
