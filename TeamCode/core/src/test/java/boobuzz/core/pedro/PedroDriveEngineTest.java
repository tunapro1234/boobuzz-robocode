package boobuzz.core.pedro;

import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Intent;
import boobuzz.core.engine.C1DriveEngine;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.mechanism.Mechanism;

import org.junit.Before;
import org.junit.Test;

import java.io.InputStream;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;

public class PedroDriveEngineTest {

    private Mechanism mechanism;

    @Before
    public void setUp() {
        try (InputStream in = getClass().getResourceAsStream("/mechanism-test.yaml")) {
            mechanism = Mechanism.load(in, "mechanism-test.yaml");
        } catch (Exception e) {
            throw new AssertionError(e);
        }
    }

    @Test
    public void manualC1IleBitBitAyniEylemiUretir() {
        Intent intent = Intent.of(new Drive.Manual(0.8, -0.6, 0.35));
        RobotAction c1 = new C1DriveEngine(mechanism).act(intent);
        RobotAction pedro = new PedroDriveEngine(mechanism).act(intent);

        assertEquals(c1, pedro);
    }

    @Test
    public void bilinmeyenYolAcikHataVerir() {
        PedroDriveEngine engine = new PedroDriveEngine(mechanism);
        assertThrows(IllegalArgumentException.class,
                () -> engine.act(Intent.of(new Drive.FollowPath("yok"))));
    }
}
