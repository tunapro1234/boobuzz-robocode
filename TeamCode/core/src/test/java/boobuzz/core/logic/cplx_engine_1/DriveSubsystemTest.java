package boobuzz.core.logic.cplx_engine_1;

import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Intent;
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
import static org.junit.Assert.assertTrue;

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
        DriveSubsystem drive = new DriveSubsystem(mechanism, new PathRegistry());
        RobotAction.Builder out = new RobotAction.Builder();
        drive.update(Intent.of(new Drive.Manual(0.5, -0.25, 0.0)), out);

        RobotAction action = out.build();
        assertEquals(0.75, action.motor("fl"), 1e-9);
        assertEquals(0.25, action.motor("fr"), 1e-9);
        assertEquals(0.25, action.motor("bl"), 1e-9);
        assertEquals(0.75, action.motor("br"), 1e-9);
    }

    @Test
    public void goToHoldVelocityGecisleriMotorEylemineYansir() {
        DriveSubsystem drive = new DriveSubsystem(mechanism, new PathRegistry());
        drive.observe(state(0L, 0.0));

        RobotAction goTo = update(drive, new Drive.GoTo(
                new Pose(24.0, 0.0, 0.0), Drive.Constraints.defaults()));
        assertTrue(goTo.motors().values().stream().anyMatch(power -> Math.abs(power) > 1e-9));

        RobotAction hold = update(drive, Drive.HOLD);
        assertTrue(hold.motors().values().stream().allMatch(power -> Math.abs(power) < 1e-9));

        RobotAction velocity = update(drive, new Drive.Velocity(10.0, -2.0, 0.1));
        assertTrue(velocity.motors().values().stream().allMatch(power -> Math.abs(power) < 1e-9));
    }

    @Test
    public void ayniKomutAyniMotorEyleminiKorur() {
        DriveSubsystem drive = new DriveSubsystem(mechanism, new PathRegistry());
        Drive.FollowPath first = new Drive.FollowPath("test-line");
        Drive.FollowPath equalButDistinct = new Drive.FollowPath("test-line");

        RobotAction firstAction = update(drive, first);
        RobotAction secondAction = update(drive, equalButDistinct);

        assertEquals(firstAction, secondAction);
    }

    @Test
    public void esitZamanliOrnekKararliMotorEylemiUretir() {
        DriveSubsystem drive = new DriveSubsystem(mechanism, new PathRegistry());
        Drive.GoTo goTo = new Drive.GoTo(
                new Pose(24.0, 0.0, 0.0), Drive.Constraints.defaults());

        drive.observe(state(100L, 0.0));
        RobotAction first = update(drive, goTo);
        drive.observe(state(100L, 0.0));
        RobotAction sameTime = update(drive, goTo);

        assertEquals(first, sameTime);
    }

    @Test
    public void artanDurumZamaniYeniMotorEylemiUretir() {
        DriveSubsystem drive = new DriveSubsystem(mechanism, new PathRegistry());
        Drive.GoTo goTo = new Drive.GoTo(
                new Pose(24.0, 0.0, 0.0), Drive.Constraints.defaults());

        drive.observe(state(100L, 0.0));
        RobotAction first = update(drive, goTo);
        drive.observe(state(125L, 1.0));
        RobotAction later = update(drive, goTo);

        assertTrue(!first.equals(later));
    }

    private static RobotAction.Builder output() {
        return new RobotAction.Builder();
    }

    private static RobotAction update(DriveSubsystem drive, Drive command) {
        RobotAction.Builder output = output();
        drive.update(Intent.of(command), output);
        return output.build();
    }

    private static RobotState state(long timestampMs, double x) {
        return new RobotState(timestampMs, Map.of(), Map.of(), 0.0,
                new Pose(x, 0.0, 0.0), 12.0);
    }
}
