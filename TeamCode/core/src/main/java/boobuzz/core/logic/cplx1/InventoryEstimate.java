package boobuzz.core.logic.cplx1;

import boobuzz.core.hal.RobotConstants;

/**
 * Honest stored-ball estimate for an unsensed intake.
 *
 * <p>The active intake has no ball sensor, so motor commands alone never confirm a
 * capture or an empty storage. The estimate is a count range plus a confidence. Command
 * observations can only keep or widen the range; only {@link #recordEvidence} (a real
 * observation with a time) can narrow it. This class never reads simulator truth.
 */
public final class InventoryEstimate {

    /** How much the range is backed by an actual observation. */
    public enum Confidence { UNKNOWN, ESTIMATED, OBSERVED }

    /** Archive HardwareConstants.Intake.maxBallCapacity. */
    public static final int CAPACITY = (int) RobotConstants.INTAKE_CAPACITY;

    private int min;
    private int max;
    private Confidence confidence;
    private long lastEvidenceMs;

    public InventoryEstimate() {
        reset();
    }

    /** Forget everything: storage may hold anything from empty to full. */
    public void reset() {
        min = 0;
        max = CAPACITY;
        confidence = Confidence.UNKNOWN;
        lastEvidenceMs = -1L;
    }

    /** Inward intake power was commanded: up to capacity may now be stored. */
    public void onIntakeInward() {
        widen(min, CAPACITY);
    }

    /** Outward intake power was commanded: stored balls may have been released. */
    public void onIntakeOutward() {
        widen(0, max);
    }

    /** A feeder pulse finished: at most one ball may have left, none is proven gone. */
    public void onFeedPulseCompleted() {
        widen(Math.max(0, min - 1), max);
    }

    /** A real observation of the stored count at HAL time {@code tMs}. */
    public void recordEvidence(int count, long tMs) {
        if (count < 0 || count > CAPACITY) {
            throw new IllegalArgumentException("observed count out of range: " + count);
        }
        min = count;
        max = count;
        confidence = Confidence.OBSERVED;
        lastEvidenceMs = tMs;
    }

    /**
     * Maximum feeder pulses an operator feed request may schedule: the upper bound of
     * the range, so an unknown storage never leads to endless dry feeding.
     */
    public int feedPulseLimit() {
        return max;
    }

    public int min() {
        return min;
    }

    public int max() {
        return max;
    }

    public Confidence confidence() {
        return confidence;
    }

    /** HAL time of the last real observation, or -1 when there has been none. */
    public long lastEvidenceMs() {
        return lastEvidenceMs;
    }

    public boolean isKnown() {
        return min == max && confidence == Confidence.OBSERVED;
    }

    private void widen(int newMin, int newMax) {
        int nextMin = Math.min(min, newMin);
        int nextMax = Math.max(max, newMax);
        if (nextMin == min && nextMax == max) {
            return;
        }
        min = nextMin;
        max = nextMax;
        if (confidence == Confidence.OBSERVED) {
            confidence = Confidence.ESTIMATED;
        }
        if (min == 0 && max == CAPACITY) {
            confidence = Confidence.UNKNOWN;
        }
    }
}
