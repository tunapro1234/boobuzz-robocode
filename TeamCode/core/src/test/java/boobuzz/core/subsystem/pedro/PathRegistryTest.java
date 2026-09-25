package boobuzz.core.subsystem.pedro;

import boobuzz.core.hal.Mechanism;

import com.pedropathing.follower.Follower;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public class PathRegistryTest {

    private Follower follower;
    private PathRegistry registry;

    @Before
    public void setUp() {
        Mechanism mechanism = Mechanism.DEFAULT;
        HalLocalizer localizer = new HalLocalizer();
        HalDrivetrain drivetrain = new HalDrivetrain(PedroDrive.wheelNames(mechanism));
        follower = PedroConstants.createFollower(mechanism, localizer, drivetrain, () -> 0L);
        registry = new PathRegistry();
    }

    @Test
    public void startsKnownTestLinePath() {
        registry.start(follower, "test-line");

        assertTrue(follower.following());
        assertEquals(120.0, follower.currentPath().endPose().x(), 1e-9);
        assertEquals(72.0, follower.currentPath().endPose().y(), 1e-9);
    }

    @Test
    public void testTurnKeepsTargetAtOneHundredTwentySeventyTwo() {
        // test-turn position is fixed at (120, 72), the same as test-line's end.
        registry.start(follower, "test-turn");

        assertTrue(follower.holding());
        assertEquals(120.0, follower.poseAt(0).x(), 1e-9);
        assertEquals(72.0, follower.poseAt(0).y(), 1e-9);
        assertEquals(Math.PI / 2.0, follower.poseAt(0).heading(), 1e-9);
    }

    @Test
    public void unknownPathFails() {
        assertThrows(IllegalArgumentException.class,
                () -> registry.start(follower, "unknown"));
    }
}
