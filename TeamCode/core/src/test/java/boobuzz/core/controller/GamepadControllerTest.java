package boobuzz.core.controller;

import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.contract.GamepadSource;
import boobuzz.core.contract.GamepadState;

import com.pedropathing.math.Pose;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/** Field-oriented drive: conversion ends in L3, and Drive remains robot-frame. */
public class GamepadControllerTest {

    private static final double EPS = 1e-9;

    /** Test gamepad. */
    private static final class Pad implements GamepadSource {
        GamepadState state = GamepadState.neutral();

        @Override public GamepadState get() { return state; }
    }

    /** ly is negative when pushed up; this is the "forward stick." */
    private static GamepadState forwardStick() {
        return new GamepadState(0, -1, 0, 0,
                false, false, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
    }

    private static Feedback at(double heading) {
        return Feedback.of(new WorldSnapshot(0, new Pose(72, 72, heading), heading, 12.6));
    }

    private static Drive.Manual driveOf(Intent intent) {
        assertTrue("expected Drive.Manual", intent.drive() instanceof Drive.Manual);
        return (Drive.Manual) intent.drive();
    }

    @Test
    public void fieldOrientedIsOnByDefault() {
        Pad pad = new Pad();
        pad.state = forwardStick();
        Drive.Manual d = driveOf(new GamepadController(pad).decide(at(Math.PI / 2)));
        assertEquals(0.0, d.vx(), EPS);
        assertEquals(-1.0, d.vy(), EPS);
    }

    @Test
    public void forwardStickUnchangedAtHeadingZero() {
        Pad pad = new Pad();
        pad.state = forwardStick();
        Drive.Manual d = driveOf(new GamepadController(pad).decide(at(0)));
        assertEquals(1.0, d.vx(), EPS);
        assertEquals(0.0, d.vy(), EPS);
    }

    @Test
    public void forwardStickRotatesToVyAtHalfPiHeading() {
        Pad pad = new Pad();
        pad.state = forwardStick();
        Drive.Manual d = driveOf(new GamepadController(pad).decide(at(Math.PI / 2)));
        // The robot faces field +y; field +x is robot RIGHT, so vy is negative.
        assertEquals(0.0, d.vx(), EPS);
        assertEquals(-1.0, d.vy(), EPS);
    }

    @Test
    public void headingIgnoredInRobotOrientedMode() {
        Pad pad = new Pad();
        GamepadController c = new GamepadController(pad);
        pad.state = new GamepadState(0, -1, 0, 0,
                false, true, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
        Drive.Manual d = driveOf(c.decide(at(Math.PI / 2)));
        assertEquals(1.0, d.vx(), EPS);
        assertEquals(0.0, d.vy(), EPS);
    }

    @Test
    public void toggleTriggersOnEdgeNotWhileHeld() {
        Pad pad = new Pad();
        GamepadController c = new GamepadController(pad);
        GamepadState bHeld = new GamepadState(0, -1, 0, 0,
                false, true, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
        pad.state = bHeld;
        assertEquals(1.0, driveOf(c.decide(at(Math.PI / 2))).vx(), EPS);
        assertEquals(1.0, driveOf(c.decide(at(Math.PI / 2))).vx(), EPS);
        assertEquals(1.0, driveOf(c.decide(at(Math.PI / 2))).vx(), EPS);
        pad.state = GamepadState.neutral();
        c.decide(at(0));
        pad.state = bHeld;
        Drive.Manual toggledBack = driveOf(c.decide(at(Math.PI / 2)));
        assertEquals(0.0, toggledBack.vx(), EPS);
        assertEquals(-1.0, toggledBack.vy(), EPS);
    }

    @Test
    public void yResetMakesCurrentHeadingForward() {
        Pad pad = new Pad();
        GamepadController c = new GamepadController(pad);
        pad.state = new GamepadState(0, -1, 0, 0,
                false, false, false, true, false, false, 0, 0, GamepadState.Dpad.NONE);
        Drive.Manual d = driveOf(c.decide(at(Math.PI / 2)));
        // Even on the same tick after reset, robot frame equals field frame.
        assertEquals(1.0, d.vx(), EPS);
        assertEquals(0.0, d.vy(), EPS);
    }

    @Test
    public void rotationIsRobotFrameInBothModes() {
        Pad pad = new Pad();
        GamepadState turn = new GamepadState(0, 0, -1, 0,
                false, false, false, false, false, false, 0, 0, GamepadState.Dpad.NONE);
        pad.state = turn;
        assertEquals(1.0, driveOf(new GamepadController(pad).decide(at(Math.PI / 3))).omega(), EPS);
    }

    @Test
    public void yawUsedWhenPoseMissing() {
        Pad pad = new Pad();
        pad.state = forwardStick();
        Feedback noPose = Feedback.of(new WorldSnapshot(0, null, Math.PI / 2, 12.6));
        Drive.Manual d = driveOf(new GamepadController(pad).decide(noPose));
        assertEquals(0.0, d.vx(), EPS);
        assertEquals(-1.0, d.vy(), EPS);
    }
}
