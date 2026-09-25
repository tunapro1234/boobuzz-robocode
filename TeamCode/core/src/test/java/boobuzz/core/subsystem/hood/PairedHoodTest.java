package boobuzz.core.subsystem.hood;

import boobuzz.core.contract.ActionValidator;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.RobotConstants;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class PairedHoodTest {

    private static final double EPS = 1e-9;
    private static final String LEFT = RobotConstants.HOOD_LEFT_SERVO_NAME;
    private static final String RIGHT = RobotConstants.HOOD_RIGHT_SERVO_NAME;

    @Test
    public void archiveComplementaryEndpoints() {
        assertPair(25.0, 1.0, 0.0);
        assertPair(44.0, 0.4553333333, 0.5446666667);
        assertPair(45.0, 0.4266666667, 0.5733333333);
        assertPair(50.0, 0.2833333333, 0.7166666667);
    }

    @Test
    public void clipsToMechanismRangeAndRejectsNonFinite() {
        assertPair(10.0, 1.0, 0.0);
        assertPair(80.0, 0.2833333333, 0.7166666667);
        try {
            new PairedHood().setAngleDeg(Double.NaN);
            fail("NaN angle must throw");
        } catch (IllegalArgumentException expected) {
            // expected
        }
    }

    @Test
    public void presetsComeFromArchive() {
        assertEquals(44.0, RobotConstants.HOOD_DEFAULT_DEG, 0.0);
        assertEquals(25.0, RobotConstants.HOOD_STOW_DEG, 0.0);
        assertEquals(45.0, RobotConstants.HOOD_RECOVERY_DEG, 0.0);
        // Left inversion lives in the 1-u mapping only, never a second HAL reversal.
        assertEquals("FORWARD", RobotConstants.HOOD_LEFT.direction());
        assertEquals("FORWARD", RobotConstants.HOOD_RIGHT.direction());
    }

    @Test
    public void initWritesNothingAndReportsNoAngle() {
        PairedHood hood = new PairedHood();
        for (long t = 0; t <= 1000; t += 20) {
            RobotAction action = tick(hood, t);
            assertTrue("no servo write before first command", action.servos().isEmpty());
        }
        assertFalse(hood.isCommanded());
        assertTrue(Double.isNaN(hood.commandedAngleDeg()));
        assertFalse(hood.settled());
    }

    @Test
    public void bothServosInEveryCommandedFrameAndValidatorAccepts() {
        PairedHood hood = new PairedHood();
        hood.setAngleDeg(44.0);
        for (long t = 0; t <= 400; t += 20) {
            RobotAction action = tick(hood, t);
            assertEquals(2, action.servos().size());
            assertEquals(1.0, action.servo(LEFT) + action.servo(RIGHT), EPS);
            ActionValidator.validate(action, Mechanism.DEFAULT);
        }
    }

    @Test
    public void unknownStartBudgetsWorstCaseTravel() {
        PairedHood hood = new PairedHood();
        hood.setAngleDeg(44.0);
        tick(hood, 1000);
        // 25 deg at 90 deg/s = 278 ms (ceil) + 100 ms margin.
        assertEquals(1000L + 278L + 100L, hood.settleAtMs());
        tick(hood, 1360);
        assertFalse(hood.settled());
        tick(hood, 1380);
        assertTrue(hood.settled());
    }

    @Test
    public void knownStartUsesActualTravelAndInterruptRecomputes() {
        PairedHood hood = new PairedHood();
        hood.setAngleDeg(25.0);
        tick(hood, 0);
        tick(hood, 400);
        assertTrue(hood.settled());

        hood.setAngleDeg(43.0);
        tick(hood, 400);
        // 18 deg -> 200 ms + 100 ms margin.
        assertEquals(700L, hood.settleAtMs());
        assertFalse(hood.settled());

        // Interrupt after 100 ms: estimated 34 deg, new target 25 deg -> 9 deg = 100 ms.
        tick(hood, 480);
        hood.setAngleDeg(25.0);
        tick(hood, 500);
        assertEquals(500L + 100L + 100L, hood.settleAtMs());
        tick(hood, 680);
        assertFalse(hood.settled());
        tick(hood, 700);
        assertTrue(hood.settled());
    }

    @Test
    public void repeatedSameTargetDoesNotRestartSettle() {
        PairedHood hood = new PairedHood();
        hood.setAngleDeg(44.0);
        tick(hood, 0);
        long settleAt = hood.settleAtMs();
        hood.setAngleDeg(44.0);
        tick(hood, 200);
        assertEquals(settleAt, hood.settleAtMs());
    }

    private static void assertPair(double angle, double left, double right) {
        PairedHood hood = new PairedHood();
        hood.setAngleDeg(angle);
        RobotAction action = tick(hood, 0);
        assertEquals("left at " + angle, left, action.servo(LEFT), EPS);
        assertEquals("right at " + angle, right, action.servo(RIGHT), EPS);
    }

    private static RobotAction tick(PairedHood hood, long t) {
        hood.observe(new RobotState(t, Map.of(), Map.of(), 0.0, new Pose(0.0, 0.0, 0.0), 12.0));
        RobotAction.Builder out = new RobotAction.Builder();
        hood.update(out);
        return out.build();
    }
}
