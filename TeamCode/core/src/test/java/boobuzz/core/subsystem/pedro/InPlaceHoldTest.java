package boobuzz.core.subsystem.pedro;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.Mechanism;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * An in-place turn (same x/y, new heading) runs as a Pedro hold. TURN_TO is done
 * "when the in-place path is done" (phase-1.1 gamepad-map-analysis), so the hold
 * must report busy until it settles or its timeout ends it, never on its first tick.
 */
public class InPlaceHoldTest {

    private static final long DT_MS = 20L;
    private static final double X = 10.0;
    private static final double Y = 10.0;
    private static final double TARGET = Math.PI / 2.0;

    @Test
    public void turnOnFreshDriveStaysBusyUntilHeadingIsReached() {
        PedroDrive drive = new PedroDrive(Mechanism.DEFAULT, new PathRegistry());
        tick(drive, 0L, 0.0);
        drive.turnTo(TARGET);
        tick(drive, 20L, 0.0);
        assertFalse("turn reported done before turning", drive.pathDone());
        tick(drive, 40L, 0.0);
        assertFalse(drive.pathDone());

        // Heading reached and the robot is still: Pedro's hold conditions end it.
        tick(drive, 60L, TARGET);
        assertTrue(drive.pathDone());
    }

    @Test
    public void turnAfterAFinishedTurnIsBusyAgain() {
        PedroDrive drive = new PedroDrive(Mechanism.DEFAULT, new PathRegistry());
        tick(drive, 0L, 0.0);
        drive.turnTo(TARGET);
        tick(drive, 20L, 0.0);
        tick(drive, 40L, TARGET);
        assertTrue(drive.pathDone());

        drive.turnTo(0.0);
        tick(drive, 60L, TARGET);
        assertFalse("second turn reported done before turning", drive.pathDone());
    }

    @Test
    public void namedHoldStartsBusy() {
        PedroDrive drive = new PedroDrive(Mechanism.DEFAULT, new PathRegistry());
        // test-turn holds (120, 72, pi/2); start there facing 0.
        observe(drive, 0L, 120.0, 72.0, 0.0);
        update(drive);
        drive.follow(PathRequest.named("test-turn"));
        observe(drive, DT_MS, 120.0, 72.0, 0.0);
        update(drive);
        assertFalse("named hold reported done before turning", drive.pathDone());
    }

    private static void tick(PedroDrive drive, long t, double heading) {
        observe(drive, t, X, Y, heading);
        update(drive);
    }

    private static void observe(PedroDrive drive, long t, double x, double y, double heading) {
        drive.observe(new RobotState(t, Map.of(), Map.of(), heading,
                new Pose(x, y, heading), 12.0));
    }

    private static void update(PedroDrive drive) {
        RobotAction.Builder out = new RobotAction.Builder();
        drive.update(out);
    }
}
