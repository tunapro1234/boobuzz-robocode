package boobuzz.core.contract;

import java.util.List;

/**
 * DOWNWARD intent. The controller produces it; the engine consumes it.
 *
 * <p>Passed through rather than global: (a) direct RL action, (b) recordable for
 * replay, and (c) testable. It also eliminates the singleton edge case.
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
