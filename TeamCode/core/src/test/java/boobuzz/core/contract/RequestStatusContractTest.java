package boobuzz.core.contract;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class RequestStatusContractTest {

    @Test
    public void lifecycleContainsActiveAndAllTerminalStates() {
        assertFalse(new RequestStatus(1, RequestStatus.State.ACTIVE, 0.0, "").terminal());
        assertTrue(RequestStatus.done(1).terminal());
        assertTrue(new RequestStatus(1, RequestStatus.State.FAILED, 0.0, "").terminal());
        assertTrue(RequestStatus.rejected(1, "cancelled").terminal());
    }
}
