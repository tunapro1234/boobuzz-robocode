package boobuzz.core.subsystem.pedro;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.Mechanism;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.Paths;
import com.pedropathing.config.Modifier;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/** Pedro 3.0 settings that must reproduce the archive's Pedro 2.0.4 behaviour. */
public class PedroArchiveParityTest {

    /** Pedro core 3.0.0 ForesightConfig.maxBrakingPower default (javap: ldc 0.2d). */
    private static final double PEDRO_DEFAULT_MAX_BRAKING_POWER = 0.2;

    @Test
    public void pathEndToleranceIsTheArchiveTValue() {
        ForesightConfig config = PedroConstants.createForesightConfig(Mechanism.DEFAULT);
        // Archive PathConstraints(0.99, ...): the path ends at t >= 0.99.
        assertEquals(1.0 - 0.99, config.parametricTConstraint.get(), 1e-12);
    }

    @Test
    public void brakingStrengthMapsOnlyToDecelerationScale() {
        PedroDrive drive = new PedroDrive(Mechanism.DEFAULT, new PathRegistry());
        observe(drive, 0L, new Pose(0.0, 0.0, 0.0));
        PathRequest request = PathRequest.path(java.util.List.of(
                        PathRequest.line(new Pose(24.0, 0.0, 0.0))))
                .withBraking(new PathRequest.Braking(0.85, 0.9));
        Path path = drive.withHeadingAndConstraints(
                Paths.line(new Pose(0.0, 0.0, 0.0), new Pose(24.0, 0.0, 0.0)), request);
        ForesightConfig config = ((Foresight) drive.follower().algorithm()).config;

        assertEquals(1, path.modifiers.size());
        for (Modifier modifier : path.modifiers) {
            modifier.apply();
        }
        try {
            assertEquals(0.85, config.maxDecelerationScale.get(), 0.0);
            assertEquals(PEDRO_DEFAULT_MAX_BRAKING_POWER, config.maxBrakingPower.get(), 0.0);
        } finally {
            for (Modifier modifier : path.modifiers) {
                modifier.revert();
            }
        }
    }

    @Test
    public void inPlaceTurnUsesNoHoldScaling() {
        PedroDrive drive = new PedroDrive(Mechanism.DEFAULT, new PathRegistry());
        double headingError = 0.2;
        observe(drive, 0L, new Pose(10.0, 10.0, 0.0));
        update(drive);
        observe(drive, 20L, new Pose(10.0, 10.0, 0.0));
        drive.turnTo(headingError);
        RobotAction action = update(drive);

        // Unscaled P heading output; Pedro's holdPointHeadingScaling (0.35) would
        // shrink it to 0.07.
        double expected = headingError * PedroConstants.HEADING_KP;
        assertEquals(-expected, action.motor("fl"), 1e-9);
        assertEquals(expected, action.motor("fr"), 1e-9);
        assertEquals(-expected, action.motor("bl"), 1e-9);
        assertEquals(expected, action.motor("br"), 1e-9);
    }

    @Test
    public void pathWithoutHoldEndReportsDone() {
        PedroDrive drive = new PedroDrive(Mechanism.DEFAULT, new PathRegistry());
        observe(drive, 0L, new Pose(0.0, 0.0, 0.0));
        update(drive);
        observe(drive, 20L, new Pose(0.0, 0.0, 0.0));
        drive.follow(PathRequest.goTo(new Pose(24.0, 0.0, 0.0),
                PathRequest.Constraints.defaults()).withHoldEnd(false));
        update(drive);
        assertFalse(drive.pathDone());

        boolean done = false;
        for (long t = 40L; t <= 200L && !done; t += 20L) {
            observe(drive, t, new Pose(24.0, 0.0, 0.0));
            update(drive);
            done = drive.pathDone();
        }
        assertTrue("holdEnd=false path never reported done", done);
    }

    private static void observe(PedroDrive drive, long t, Pose pose) {
        drive.observe(new RobotState(t, Map.of(), Map.of(), pose.heading(), pose, 12.0));
    }

    private static RobotAction update(PedroDrive drive) {
        RobotAction.Builder out = new RobotAction.Builder();
        drive.update(out);
        return out.build();
    }
}
