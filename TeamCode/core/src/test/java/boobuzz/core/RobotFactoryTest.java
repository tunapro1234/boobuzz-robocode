package boobuzz.core;

import boobuzz.core.contract.GamepadState;
import boobuzz.core.hal.IHal;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.logic.EngineRegistry;
import boobuzz.core.logic.cplx1.CplxEngine1;
import boobuzz.core.logic.direct.DirectEngine;
import boobuzz.core.subsystem.intake.PowerIntake;

import com.pedropathing.math.Pose;

import org.junit.Before;
import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class RobotFactoryTest {

    private Mechanism mechanism;

    @Before
    public void setUp() {
        mechanism = Mechanism.DEFAULT;
    }

    @Test
    public void engineIsBuiltByFactory() {
        StubHal hal = new StubHal();
        assertEquals("cplx1",
                RobotFactory.create(hal, mechanism).engine().name());
    }

    @Test
    public void replayAliasKeepsCplx1Binding() {
        StubHal hal = new StubHal();
        assertEquals("cplx1", RobotFactory.create(hal, mechanism,
                EngineRegistry.CPLX1_ALIAS).engine().name());
    }

    @Test
    public void directSelectorUsesStableIndexZero() {
        StubHal hal = new StubHal();
        assertEquals(EngineRegistry.DIRECT_INDEX,
                EngineRegistry.indexForName(EngineRegistry.DIRECT_NAME));
        assertEquals("direct", RobotFactory.create(hal, mechanism,
                EngineRegistry.DIRECT_NAME).engine().name());
    }

    @Test
    public void futureSelectorsAreReservedUntilImplemented() {
        assertFalse(EngineRegistry.isImplementedIndex(EngineRegistry.CPLX2_INDEX));
        assertFalse(EngineRegistry.isImplementedIndex(EngineRegistry.VISION_A_INDEX));
        assertFalse(EngineRegistry.isImplementedIndex(EngineRegistry.VISION_B_INDEX));
        assertFalse(EngineRegistry.isImplementedIndex(EngineRegistry.RANGE_INDEX));
    }

    @Test
    public void factoryWiresPowerIntakeIntoBothEngines() {
        StubHal hal = new StubHal();
        assertTrue(((DirectEngine) RobotFactory.create(hal, mechanism,
                EngineRegistry.DIRECT_NAME).engine()).subsystems().intake()
                instanceof PowerIntake);
        assertTrue(((CplxEngine1) RobotFactory.create(hal, mechanism,
                EngineRegistry.CPLX1_NAME).engine()).subsystems().intake()
                instanceof PowerIntake);
    }

    private static final class StubHal implements IHal {
        @Override public long now() { return 0; }

        @Override public RobotState read() {
            return new RobotState(0, Map.of(), Map.of(), 0.0,
                    new Pose(0.0, 0.0, 0.0), 12.0);
        }

        @Override public void write(RobotAction action) {}

        @Override public GamepadState get() { return GamepadState.neutral(); }
    }
}
