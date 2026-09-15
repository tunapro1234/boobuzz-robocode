package boobuzz.core.logic.cplx_engine_1;

import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Intent;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.MechanismLoader;

import org.junit.Before;
import org.junit.Test;

import java.io.InputStream;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;

public class CplxEngine1Test {

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
    public void manualMecanumGucunuDogruUretir() {
        Intent intent = Intent.of(new Drive.Manual(0.8, -0.6, 0.35));
        RobotAction pedro = new CplxEngine1(mechanism).act(intent);

        assertEquals(1.05 / 1.75, pedro.motor("fl"), 1e-9);
        assertEquals(0.55 / 1.75, pedro.motor("fr"), 1e-9);
        assertEquals(-0.15 / 1.75, pedro.motor("bl"), 1e-9);
        assertEquals(1.75 / 1.75, pedro.motor("br"), 1e-9);
    }

    @Test
    public void bilinmeyenYolAcikHataVerir() {
        CplxEngine1 engine = new CplxEngine1(mechanism);
        assertThrows(IllegalArgumentException.class,
                () -> engine.act(Intent.of(new Drive.FollowPath("yok"))));
    }
}
