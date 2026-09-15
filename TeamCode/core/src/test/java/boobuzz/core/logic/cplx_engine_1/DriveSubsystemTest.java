package boobuzz.core.logic.cplx_engine_1;

import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Intent;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.MechanismLoader;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.junit.Before;
import org.junit.Test;

import java.io.InputStream;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
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

    @Test
    public void goToHoldVelocityGecisleriFollowerModunuDegistirir() {
        DriveSubsystem drive = DriveSubsystem.pedro(mechanism, new PathRegistry());

        drive.update(Intent.of(new Drive.GoTo(
                new Pose(24.0, 12.0, 0.5), Drive.Constraints.defaults())), output());
        assertTrue(drive.activeCommand() instanceof Drive.GoTo);
        assertEquals(Follower.Mode.HOLD, drive.followerMode());

        drive.update(Intent.of(Drive.HOLD), output());
        assertTrue(drive.activeCommand() instanceof Drive.Hold);
        assertEquals(Follower.Mode.HOLD, drive.followerMode());

        drive.update(Intent.of(new Drive.Velocity(10.0, -2.0, 0.1)), output());
        assertTrue(drive.activeCommand() instanceof Drive.Velocity);
        assertEquals(Follower.Mode.IDLE, drive.followerMode());
    }

    @Test
    public void ayniKomutFolloweriYenidenBaslatmaz() {
        DriveSubsystem drive = DriveSubsystem.pedro(mechanism, new PathRegistry());
        Drive.FollowPath first = new Drive.FollowPath("test-line");
        Drive.FollowPath equalButDistinct = new Drive.FollowPath("test-line");

        drive.update(Intent.of(first), output());
        drive.update(Intent.of(equalButDistinct), output());

        assertSame(first, drive.activeCommand());
        assertEquals(Follower.Mode.FOLLOW, drive.followerMode());
    }

    @Test
    public void ilkVeEsitZamanliOrneklerdeDtSifirdir() {
        DriveSubsystem drive = DriveSubsystem.pedro(mechanism, new PathRegistry());

        drive.observe(10L, state(100L));
        assertEquals(0.0, drive.deltaTimeSeconds(), 1e-9);
        drive.observe(20L, state(100L));
        assertEquals(0.0, drive.deltaTimeSeconds(), 1e-9);
    }

    @Test
    public void artanDurumZamaniSaniyeyeCevrilir() {
        DriveSubsystem drive = DriveSubsystem.pedro(mechanism, new PathRegistry());

        drive.observe(999L, state(100L));
        drive.observe(1_999L, state(125L));

        assertEquals(0.025, drive.deltaTimeSeconds(), 1e-9);
    }

    private static RobotAction.Builder output() {
        return new RobotAction.Builder();
    }

    private static RobotState state(long timestampMs) {
        return new RobotState(timestampMs, Map.of(), Map.of(), 0.0,
                new Pose(0.0, 0.0, 0.0), 12.0);
    }
}
