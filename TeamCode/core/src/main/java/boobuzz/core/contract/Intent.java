package boobuzz.core.contract;

import java.util.List;

/**
 * ASAGI giden niyet. Controller uretir, Engine tuketir.
 *
 * <p>Gecirilerek tasinir, global degil: (a) dogrudan RL eylemi, (b) replay icin
 * kaydedilebilir, (c) test edilebilir. Singleton ucunu de oldurur.
 */
public record Intent(Drive drive, List<Request> newRequests, int[] cancels) {

    public Intent {
        newRequests = List.copyOf(newRequests);
        cancels = (cancels == null) ? new int[0] : cancels.clone();
    }

    public static Intent of(Drive drive) {
        return new Intent(drive, List.of(), new int[0]);
    }

    public static Intent idle() {
        return of(Drive.HOLD);
    }
}
