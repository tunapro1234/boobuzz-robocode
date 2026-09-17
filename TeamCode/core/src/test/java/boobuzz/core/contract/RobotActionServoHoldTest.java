package boobuzz.core.contract;

import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Pins the two-map proto2 representation and sparse positional-servo command. */
public class RobotActionServoHoldTest {

    @Test
    public void omittedServoIsDistinctFromExplicitZero() {
        RobotAction omitted = RobotAction.zero();
        RobotAction explicitZero = new RobotAction(Map.of(), Map.of("hood_right", 0.0));

        assertFalse(omitted.servos().containsKey("hood_right"));
        assertTrue(explicitZero.servos().containsKey("hood_right"));
        assertTrue(explicitZero.servos().get("hood_right") == 0.0);
    }

    @Test
    public void actionHasOnlyMotorsServosAndEvents() {
        assertTrue(RobotAction.class.getRecordComponents().length == 3);
        assertTrue(RobotAction.class.getRecordComponents()[0].getName().equals("motors"));
        assertTrue(RobotAction.class.getRecordComponents()[1].getName().equals("servos"));
        assertTrue(RobotAction.class.getRecordComponents()[2].getName().equals("events"));
    }
}
