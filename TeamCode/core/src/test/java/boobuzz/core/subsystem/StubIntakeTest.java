package boobuzz.core.subsystem;

import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;

public class StubIntakeTest {

    @Test
    public void runAndStopEmitEdgeEvents() {
        StubIntake intake = new StubIntake();
        intake.observe(state(20));

        intake.run(0.75);
        RobotAction.Builder onOut = new RobotAction.Builder();
        intake.update(onOut);
        RobotAction on = onOut.build();
        assertEquals(1, on.events().size());
        assertEquals("intake.on", on.events().get(0).name());
        assertEquals(20L, on.events().get(0).tMs());

        intake.stop();
        RobotAction.Builder offOut = new RobotAction.Builder();
        intake.update(offOut);
        RobotAction off = offOut.build();
        assertEquals(1, off.events().size());
        assertEquals("intake.off", off.events().get(0).name());
        assertFalse(intake.hasBall());
    }

    private static RobotState state(long tMs) {
        return new RobotState(tMs, Map.of(), Map.of(), 0.0,
                new Pose(0.0, 0.0, 0.0), 12.6);
    }
}
