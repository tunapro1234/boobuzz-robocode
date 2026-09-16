package boobuzz.core.subsystem;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class StubShooterTest {

    @Test
    public void spinUpBecomesReadyAfterConfiguredTime() {
        StubShooter shooter = new StubShooter();
        shooter.observe(state(0));
        shooter.spinUp(300.0);

        assertFalse(shooter.isReady());
        shooter.observe(state(Math.round(RobotConstants.STUB_SPINUP_S * 1000.0) - 1));
        assertFalse(shooter.isReady());
        shooter.observe(state(Math.round(RobotConstants.STUB_SPINUP_S * 1000.0)));
        assertTrue(shooter.isReady());
    }

    @Test
    public void feedEmitsStartAndEndAtObservedTimes() {
        StubShooter shooter = new StubShooter();
        long spinupMs = Math.round(RobotConstants.STUB_SPINUP_S * 1000.0);
        long feedMs = Math.round(RobotConstants.STUB_FEED_S * 1000.0);
        shooter.observe(state(spinupMs));
        shooter.spinUp(300.0);
        shooter.observe(state(spinupMs + spinupMs));
        assertTrue(shooter.isReady());
        shooter.feed();

        RobotAction.Builder startOut = new RobotAction.Builder();
        shooter.update(startOut);
        RobotAction start = startOut.build();
        assertEquals(1, start.events().size());
        assertEquals("shooter.feed.start", start.events().get(0).name());
        assertEquals(spinupMs + spinupMs, start.events().get(0).tMs());

        shooter.observe(state(spinupMs + spinupMs + feedMs));
        RobotAction.Builder endOut = new RobotAction.Builder();
        shooter.update(endOut);
        RobotAction end = endOut.build();
        assertFalse(shooter.isFeeding());
        assertEquals(1, end.events().size());
        assertEquals("shooter.feed.end", end.events().get(0).name());
        assertEquals(spinupMs + spinupMs + feedMs, end.events().get(0).tMs());
    }

    private static RobotState state(long tMs) {
        return new RobotState(tMs, Map.of(), Map.of(), 0.0,
                new Pose(0.0, 0.0, 0.0), 12.6);
    }
}
