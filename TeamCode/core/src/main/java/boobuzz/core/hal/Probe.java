package boobuzz.core.hal;

/**
 * Ara degerleri disari sizdirma noktasi. Node framework'u DEGILDIR:
 * tek zincir, cok gozlem noktasi.
 *
 * <p>{@code NullProbe} (yarisma), {@code SocketProbe} (gorsellestirici),
 * {@code FileProbe} (replay).
 */
public interface Probe {
    <T> void publish(String name, T message, long t);
}
