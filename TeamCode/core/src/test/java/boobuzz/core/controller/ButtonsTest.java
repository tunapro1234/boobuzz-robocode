package boobuzz.core.controller;

import boobuzz.core.contract.GamepadState;

import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class ButtonsTest {

    @Test
    public void edgesAndToggleOnlyChangeOnPress() {
        GamepadState off = GamepadState.neutral();
        GamepadState on = new GamepadState(0, 0, 0, 0,
                false, false, false, false, false, true,
                0, 0, GamepadState.Dpad.NONE);
        Buttons buttons = new Buttons(off, on, 0, 20);

        assertTrue(buttons.pressed("rb"));
        assertFalse(buttons.released("rb"));
        assertTrue(buttons.toggle("rb"));
        assertTrue(buttons.toggle("rb"));

        buttons.update(on, 40);
        assertFalse(buttons.pressed("rb"));
        assertTrue(buttons.held("right_bumper"));
        assertTrue(buttons.toggle("rb"));

        buttons.update(off, 60);
        assertTrue(buttons.released("rb"));
        assertFalse(buttons.held("rb"));
    }

    @Test
    public void heldForUsesHalClockAndBackStartNames() {
        GamepadState start = new GamepadState(0, 0, 0, 0,
                false, false, false, false, false, false,
                0, 0, GamepadState.Dpad.NONE, true, true);
        Buttons buttons = new Buttons(GamepadState.neutral(), start, 0, 0);
        assertFalse(buttons.heldFor("start", 1.0));
        buttons.update(start, 999);
        assertFalse(buttons.heldFor("start", 1.0));
        buttons.update(start, 1000);
        assertTrue(buttons.heldFor("start", 1.0));
        assertTrue(buttons.held("back"));
    }
}
