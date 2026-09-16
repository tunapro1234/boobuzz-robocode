package boobuzz.core;

import boobuzz.core.hal.GamepadState;
import boobuzz.core.hal.Hal;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.MechanismLoader;

import com.pedropathing.math.Pose;

import org.junit.Before;
import org.junit.Test;

import java.io.InputStream;
import java.util.Map;

import static org.junit.Assert.assertEquals;

public class RobotFactoryTest {

    private Mechanism mechanism;

    @Before
    public void setUp() {
        try (InputStream in = getClass().getResourceAsStream("/mechanism-test.yaml")) {
            mechanism = MechanismLoader.load(in, "mechanism-test.yaml");
        } catch (Exception e) {
            throw new AssertionError(e);
        }
    }

    @Test
    public void engineIsBuiltByFactory() {
        StubHal hal = new StubHal();
        assertEquals("cplx_engine_1",
                RobotFactory.create(hal, mechanism).engine().name());
    }

    private static final class StubHal implements Hal {
        @Override public long now() { return 0; }

        @Override public RobotState read() {
            return new RobotState(0, Map.of(), Map.of(), 0.0,
                    new Pose(0.0, 0.0, 0.0), 12.0);
        }

        @Override public void write(RobotAction action) {}

        @Override public GamepadState get() { return GamepadState.neutral(); }
    }
}
