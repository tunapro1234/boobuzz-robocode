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

public class DriveSubsystemTest {

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
    public void manuelMecanumEyleminiUretir() {
        DriveSubsystem drive = DriveSubsystem.manual(mechanism);
        RobotAction.Builder out = new RobotAction.Builder();
        drive.update(Intent.of(new Drive.Manual(0.5, -0.25, 0.0)), out);

        RobotAction action = out.build();
        assertEquals(0.75, action.motor("fl"), 1e-9);
        assertEquals(0.25, action.motor("fr"), 1e-9);
        assertEquals(0.25, action.motor("bl"), 1e-9);
        assertEquals(0.75, action.motor("br"), 1e-9);
    }

    @Test
    public void pedroYoluSubsystemIcindeCozulur() {
        DriveSubsystem drive = DriveSubsystem.pedro(mechanism, new PathRegistry());
        RobotAction.Builder out = new RobotAction.Builder();

        assertThrows(IllegalArgumentException.class,
                () -> drive.update(Intent.of(new Drive.FollowPath("yok")), out));
    }
}
