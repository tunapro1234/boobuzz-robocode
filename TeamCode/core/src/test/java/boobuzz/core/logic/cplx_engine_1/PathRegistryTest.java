package boobuzz.core.logic.cplx_engine_1;

import boobuzz.core.logic.cplx_engine_1.HalDrivetrain;
import boobuzz.core.logic.cplx_engine_1.HalLocalizer;
import boobuzz.core.logic.cplx_engine_1.PathRegistry;
import boobuzz.core.logic.cplx_engine_1.PedroConstants;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.MechanismLoader;

import com.pedropathing.follower.Follower;

import org.junit.Before;
import org.junit.Test;

import java.io.InputStream;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

public class PathRegistryTest {

    private Follower follower;
    private PathRegistry registry;

    @Before
    public void setUp() {
        try (InputStream in = getClass().getResourceAsStream("/mechanism-test.yaml")) {
            Mechanism mechanism = MechanismLoader.load(in, "mechanism-test.yaml");
            HalLocalizer localizer = new HalLocalizer();
            HalDrivetrain drivetrain = new HalDrivetrain(mechanism);
            follower = PedroConstants.createFollower(localizer, drivetrain);
            registry = new PathRegistry();
        } catch (Exception e) {
            throw new AssertionError(e);
        }
    }

    @Test
    public void bilinenTestLineYolunuBaslatir() {
        registry.start(follower, "test-line");

        assertTrue(follower.following());
        assertEquals(120.0, follower.currentPath().endPose().x(), 1e-9);
        assertEquals(72.0, follower.currentPath().endPose().y(), 1e-9);
    }

    @Test
    public void testTurnAyniYerdekiYuzYirmiYetmisIkiHedefiniTutar() {
        // test-turn konumu, test-line sonuyla ayni (120, 72) olarak sabittir.
        registry.start(follower, "test-turn");

        assertTrue(follower.holding());
        assertEquals(120.0, follower.poseAt(0).x(), 1e-9);
        assertEquals(72.0, follower.poseAt(0).y(), 1e-9);
        assertEquals(Math.PI / 2.0, follower.poseAt(0).heading(), 1e-9);
    }

    @Test
    public void bilinmeyenYolHataVerir() {
        assertThrows(IllegalArgumentException.class,
                () -> registry.start(follower, "yok"));
    }
}
