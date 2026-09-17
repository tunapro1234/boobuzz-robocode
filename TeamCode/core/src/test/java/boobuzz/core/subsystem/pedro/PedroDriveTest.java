package boobuzz.core.subsystem.pedro;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.Mechanism;

import com.pedropathing.math.Pose;

import org.junit.Before;
import org.junit.Test;

import java.util.Map;
import java.util.LinkedHashMap;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class PedroDriveTest {

    private Mechanism mechanism;

    @Before
    public void setUp() {
        mechanism = Mechanism.DEFAULT;
    }

    @Test
    public void manualMecanumProducesAction() {
        PedroDrive drive = new PedroDrive(mechanism, new PathRegistry());
        RobotAction.Builder out = new RobotAction.Builder();
        drive.manual(0.5, -0.25, 0.0);
        drive.update(out);

        RobotAction action = out.build();
        assertEquals(0.75, action.motor("fl"), 1e-9);
        assertEquals(0.25, action.motor("fr"), 1e-9);
        assertEquals(0.25, action.motor("bl"), 1e-9);
        assertEquals(0.75, action.motor("br"), 1e-9);
    }

    @Test
    public void goToHoldVelocityTransitionsReachMotorAction() {
        PedroDrive drive = new PedroDrive(mechanism, new PathRegistry());
        drive.observe(state(0L, 0.0));
        drive.observe(state(20L, 0.0));

        RobotAction goTo = update(drive, PathRequest.goTo(
                new Pose(24.0, 0.0, 0.0), PathRequest.Constraints.defaults()));
        assertTrue(goTo.motors().values().stream().anyMatch(power -> Math.abs(power) > 1e-9));

        drive.stop();
        RobotAction hold = update(drive);
        assertTrue(hold.motors().values().stream().allMatch(power -> Math.abs(power) < 1e-9));

        drive.stop();
        RobotAction velocity = update(drive);
        assertTrue(velocity.motors().values().stream().allMatch(power -> Math.abs(power) < 1e-9));
    }

    @Test
    public void equalCommandPreservesMotorAction() {
        PedroDrive drive = new PedroDrive(mechanism, new PathRegistry());
        PathRequest first = PathRequest.named("test-line");
        PathRequest equalButDistinct = PathRequest.named("test-line");

        RobotAction firstAction = update(drive, first);
        RobotAction secondAction = update(drive, equalButDistinct);

        assertEquals(firstAction, secondAction);
    }

    @Test
    public void sameTimeSampleProducesDeterministicMotorAction() {
        PedroDrive drive = new PedroDrive(mechanism, new PathRegistry());
        PathRequest goTo = PathRequest.goTo(
                new Pose(24.0, 0.0, 0.0), PathRequest.Constraints.defaults());

        drive.observe(state(100L, 0.0));
        RobotAction first = update(drive, goTo);
        drive.observe(state(100L, 0.0));
        RobotAction sameTime = update(drive, goTo);

        assertEquals(first, sameTime);
    }

    @Test
    public void zeroDeltaPathUpdateEmitsFiniteZeroPower() {
        PedroDrive drive = new PedroDrive(mechanism, new PathRegistry());
        drive.observe(state(0L, 0.0));

        RobotAction action = update(drive, PathRequest.named("test-line"));

        for (double power : action.motors().values()) {
            assertTrue("zero-dt power must be finite", Double.isFinite(power));
            assertEquals(0.0, power, 1e-9);
        }
    }

    @Test
    public void laterStateTimeProducesNewMotorAction() {
        PedroDrive drive = new PedroDrive(mechanism, new PathRegistry());
        PathRequest goTo = PathRequest.goTo(
                new Pose(24.0, 0.0, 0.0), PathRequest.Constraints.defaults());

        drive.observe(state(100L, 0.0));
        RobotAction first = update(drive, goTo);
        drive.observe(state(125L, 1.0));
        RobotAction later = update(drive, goTo);

        assertTrue(!first.equals(later));
    }

    @Test
    public void repeatedInconsistentEncoderSamplesNeverEmitNonFinitePowers() {
        PedroDrive drive = new PedroDrive(mechanism, new PathRegistry());
        Map<String, Integer> encoders = new LinkedHashMap<>();
        encoders.put("fl", 697);
        encoders.put("fr", 7609);
        encoders.put("bl", 7611);
        encoders.put("br", 696);
        Map<String, Double> velocities = new LinkedHashMap<>();
        velocities.put("fl", 1550.0);
        velocities.put("fr", 1550.0);
        velocities.put("bl", 1600.0);
        velocities.put("br", 1550.0);
        PathRequest request = PathRequest.goTo(
                new Pose(120.0, 72.0, 0.0), PathRequest.Constraints.defaults());

        for (int tick = 0; tick < 200; tick++) {
            drive.observe(new RobotState(tick * 20L, encoders, velocities, -0.0023,
                    new Pose(105.25, 71.91, -0.0023), 12.0));
            if (tick == 0) {
                drive.follow(request);
            }
            RobotAction action = update(drive);
            for (double power : action.motors().values()) {
                assertTrue("power must be finite", Double.isFinite(power));
                assertTrue("power must be clamped", Math.abs(power) <= 1.0);
            }
        }
    }

    @Test
    public void nonFiniteManualInputIsZeroedBeforeHal() {
        PedroDrive drive = new PedroDrive(mechanism, new PathRegistry());
        drive.manual(Double.NaN, 0.0, 0.0);

        RobotAction action = update(drive);
        for (double power : action.motors().values()) {
            assertTrue(Double.isFinite(power));
            assertEquals(0.0, power, 1e-9);
        }
    }

    private static RobotAction.Builder output() {
        return new RobotAction.Builder();
    }

    private static RobotAction update(PedroDrive drive, PathRequest request) {
        drive.follow(request);
        RobotAction.Builder output = output();
        drive.update(output);
        return output.build();
    }

    private static RobotAction update(PedroDrive drive) {
        RobotAction.Builder output = new RobotAction.Builder();
        drive.update(output);
        return output.build();
    }

    private static RobotState state(long timestampMs, double x) {
        return new RobotState(timestampMs, Map.of(), Map.of(), 0.0,
                new Pose(x, 0.0, 0.0), 12.0);
    }
}
