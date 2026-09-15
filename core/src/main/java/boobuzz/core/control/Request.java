package boobuzz.core.control;

/**
 * Kenar tetikli istek. Seviye degil olay: "at" der, "atmaya devam et" demez.
 *
 * <p>Yasam dongusu {@link RequestStatus} ile takip edilir. Command-based
 * scheduler YOK - tek {@link Intent}'li tek donguda kaynak cakismasi yok,
 * {@code Intent}'in kendisi arbitrajdir (docs/mimari.md §4).
 */
public record Request(int id, RequestType type, double[] params) {

    public static Request of(int id, RequestType type, double... params) {
        return new Request(id, type, params);
    }

    public double param(int index, double fallback) {
        return (params != null && index < params.length) ? params[index] : fallback;
    }
}
