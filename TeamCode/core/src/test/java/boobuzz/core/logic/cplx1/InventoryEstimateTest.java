package boobuzz.core.logic.cplx1;

import org.junit.Test;

import java.lang.reflect.Field;
import java.lang.reflect.Method;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class InventoryEstimateTest {

    @Test
    public void startsUnknownOverFullCapacity() {
        InventoryEstimate estimate = new InventoryEstimate();
        assertRange(estimate, 0, 3, InventoryEstimate.Confidence.UNKNOWN);
        assertEquals(-1L, estimate.lastEvidenceMs());
        assertFalse(estimate.isKnown());
    }

    @Test
    public void commandsNeverIncreaseCertainty() {
        InventoryEstimate estimate = new InventoryEstimate();
        for (int i = 0; i < 10; i++) {
            estimate.onIntakeInward();
            estimate.onFeedPulseCompleted();
            estimate.onIntakeOutward();
            assertRange(estimate, 0, 3, InventoryEstimate.Confidence.UNKNOWN);
        }
    }

    @Test
    public void pulseAfterEvidenceOnlyWidensDownward() {
        InventoryEstimate estimate = new InventoryEstimate();
        estimate.recordEvidence(2, 1500L);
        assertTrue(estimate.isKnown());
        assertEquals(1500L, estimate.lastEvidenceMs());

        estimate.onFeedPulseCompleted();
        assertRange(estimate, 1, 2, InventoryEstimate.Confidence.ESTIMATED);
        estimate.onFeedPulseCompleted();
        estimate.onFeedPulseCompleted();
        estimate.onFeedPulseCompleted();
        assertRange(estimate, 0, 2, InventoryEstimate.Confidence.ESTIMATED);
        assertEquals("evidence time is not refreshed by commands", 1500L,
                estimate.lastEvidenceMs());

        estimate.onIntakeInward();
        assertRange(estimate, 0, 3, InventoryEstimate.Confidence.UNKNOWN);
    }

    @Test
    public void reverseAfterEvidenceWidensDownOnly() {
        InventoryEstimate estimate = new InventoryEstimate();
        estimate.recordEvidence(2, 100L);
        estimate.onIntakeOutward();
        assertRange(estimate, 0, 2, InventoryEstimate.Confidence.ESTIMATED);
        estimate.onIntakeInward();
        assertRange(estimate, 0, 3, InventoryEstimate.Confidence.UNKNOWN);
    }

    @Test
    public void cancelledPulsesDropCertainty() {
        // Two cancelled 200 ms pulses can move one ball in the sim (.35 s travel threshold).
        InventoryEstimate estimate = new InventoryEstimate();
        estimate.recordEvidence(2, 100L);
        estimate.onFeederForwardUnmetered();
        assertRange(estimate, 0, 2, InventoryEstimate.Confidence.ESTIMATED);
        assertFalse(estimate.isKnown());
    }

    @Test
    public void countsNeverGoNegative() {
        InventoryEstimate estimate = new InventoryEstimate();
        estimate.recordEvidence(0, 10L);
        for (int i = 0; i < 5; i++) {
            estimate.onFeedPulseCompleted();
        }
        assertRange(estimate, 0, 0, InventoryEstimate.Confidence.OBSERVED);
        try {
            estimate.recordEvidence(-1, 20L);
            fail("negative evidence must throw");
        } catch (IllegalArgumentException expected) {
            // expected
        }
        try {
            estimate.recordEvidence(4, 20L);
            fail("evidence above capacity must throw");
        } catch (IllegalArgumentException expected) {
            // expected
        }
    }

    @Test
    public void feedRequestLimitIsFinite() {
        InventoryEstimate estimate = new InventoryEstimate();
        assertEquals(3, estimate.feedPulseLimit());
        estimate.recordEvidence(1, 5L);
        assertEquals(1, estimate.feedPulseLimit());
        estimate.onFeedPulseCompleted();
        assertEquals(1, estimate.feedPulseLimit());
    }

    @Test
    public void resetReturnsUnknown() {
        InventoryEstimate estimate = new InventoryEstimate();
        estimate.recordEvidence(3, 100L);
        estimate.reset();
        assertRange(estimate, 0, 3, InventoryEstimate.Confidence.UNKNOWN);
        assertEquals(-1L, estimate.lastEvidenceMs());
    }

    @Test
    public void hasNoAccessToTruthOrBallIds() {
        for (Field field : InventoryEstimate.class.getDeclaredFields()) {
            String name = field.getName().toLowerCase();
            assertFalse(field.getName(), name.contains("truth") || name.contains("ballid") || name.endsWith("ids"));
        }
        for (Method method : InventoryEstimate.class.getDeclaredMethods()) {
            for (Class<?> type : method.getParameterTypes()) {
                assertTrue(method.getName() + " takes " + type,
                        type.isPrimitive());
            }
        }
    }

    private static void assertRange(InventoryEstimate estimate, int min, int max,
                                    InventoryEstimate.Confidence confidence) {
        assertEquals("min", min, estimate.min());
        assertEquals("max", max, estimate.max());
        assertEquals("confidence", confidence, estimate.confidence());
    }
}
