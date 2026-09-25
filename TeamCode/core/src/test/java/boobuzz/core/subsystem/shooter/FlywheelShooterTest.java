package boobuzz.core.subsystem.shooter;

import boobuzz.core.contract.Event;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.RobotConstants;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class FlywheelShooterTest {

    private static final double EPS = 1e-9;
    private static final String RIGHT = RobotConstants.SHOOTER_RIGHT_MOTOR_NAME;
    private static final String LEFT = RobotConstants.SHOOTER_LEFT_MOTOR_NAME;
    private static final String FEEDER = RobotConstants.FEEDER_MOTOR_NAME;
    private static final double TICKS_AT_4000 = 1166.6666667;

    @Test
    public void archiveConversionAndConstants() {
        assertEquals(4000.0, FlywheelShooter.wheelRpmFromTicksPerSecond(TICKS_AT_4000), 1e-5);
        assertEquals(RobotConstants.SHOOTER_FEEDBACK_ENCODER_NAME, RIGHT);
        assertEquals("REVERSE", RobotConstants.SHOOTER_RIGHT.direction());
        assertEquals("FORWARD", RobotConstants.SHOOTER_LEFT.direction());
        assertEquals("FLOAT", RobotConstants.SHOOTER_RIGHT.zeroPower());
        assertEquals("FLOAT", RobotConstants.SHOOTER_LEFT.zeroPower());
        assertEquals(1.0, RobotConstants.SHOOTER_FOLLOWER_SCALE, 0.0);
    }

    @Test
    public void noErrorFeedforwardAt4000OnBothOutputs() {
        FlywheelShooter shooter = new FlywheelShooter();
        shooter.spinUp(4000.0);
        RobotAction action = tick(shooter, 0, rpm(4000.0));
        assertEquals(0.728222, action.motor(RIGHT), 1e-6);
        assertEquals(0.728222, action.motor(LEFT), 1e-6);
    }

    @Test
    public void feedforwardThresholdAt1500() {
        FlywheelShooter below = new FlywheelShooter();
        below.spinUp(1499.0);
        assertEquals(0.0, tick(below, 0, rpm(1499.0)).motor(RIGHT), EPS);

        FlywheelShooter at = new FlywheelShooter();
        at.spinUp(1500.0);
        double expected = RobotConstants.SHOOTER_KS + RobotConstants.SHOOTER_KV * 1500.0;
        assertEquals(expected, tick(at, 0, rpm(1500.0)).motor(RIGHT), EPS);
    }

    @Test
    public void pidTermsFollowArchiveFormula() {
        FlywheelShooter shooter = new FlywheelShooter();
        shooter.spinUp(4000.0);
        tick(shooter, 0, rpm(3900.0));
        RobotAction action = tick(shooter, 20, rpm(3850.0));
        // error 150 (inside the 250 zone): integral 150*.02 = 3; derivative (150-100)/.02.
        double pid = RobotConstants.SHOOTER_KP * 150.0 + RobotConstants.SHOOTER_KI * 3.0
                + RobotConstants.SHOOTER_KD * 2500.0;
        assertEquals(pid + 0.728222, action.motor(RIGHT), 1e-6);
        assertEquals(3.0, shooter.integralAccum(), 1e-6);
    }

    @Test
    public void integralZoneResetAndClamp() {
        FlywheelShooter shooter = new FlywheelShooter();
        shooter.spinUp(4000.0);
        tick(shooter, 0, rpm(3800.0));
        tick(shooter, 20, rpm(3800.0));
        assertTrue(shooter.integralAccum() > 0.0);
        tick(shooter, 40, rpm(3000.0));
        assertEquals("outside the zone the integral resets", 0.0, shooter.integralAccum(), EPS);

        for (long t = 60; t <= 200_000; t += 20) {
            tick(shooter, t, rpm(3760.0));
        }
        assertEquals(RobotConstants.SHOOTER_INTEGRAL_MAX_ACCUM, shooter.integralAccum(), EPS);
    }

    @Test
    public void saturationClipsBothOutputsAndRecovers() {
        FlywheelShooter shooter = new FlywheelShooter();
        shooter.spinUp(4000.0);
        RobotAction low = tick(shooter, 0, rpm(0.0));
        assertEquals(1.0, low.motor(RIGHT), EPS);
        assertEquals(1.0, low.motor(LEFT), EPS);
        RobotAction high = tick(shooter, 20, rpm(9000.0));
        assertEquals(-1.0, high.motor(RIGHT), EPS);
        assertEquals(-1.0, high.motor(LEFT), EPS);
        tick(shooter, 40, rpm(4000.0));
        RobotAction settled = tick(shooter, 60, rpm(4000.0));
        assertTrue(Math.abs(settled.motor(RIGHT) - 0.728222) < 0.01);
    }

    @Test
    public void readyNeeds150msWithinToleranceAndDropsOnDip() {
        FlywheelShooter shooter = new FlywheelShooter();
        shooter.spinUp(4000.0);
        tick(shooter, 0, rpm(3950.0));
        tick(shooter, 140, rpm(3950.0));
        assertFalse(shooter.isReady());
        tick(shooter, 160, rpm(4050.0));
        assertTrue(shooter.isReady());

        tick(shooter, 180, rpm(3890.0));
        assertFalse("a >100 RPM dip drops readiness", shooter.isReady());
        tick(shooter, 200, rpm(3990.0));
        tick(shooter, 340, rpm(3990.0));
        assertFalse(shooter.isReady());
        tick(shooter, 360, rpm(3990.0));
        assertTrue(shooter.isReady());
    }

    @Test
    public void feedIsOneArchivePulseAndOnlyWhenReady() {
        FlywheelShooter shooter = new FlywheelShooter();
        shooter.spinUp(4000.0);
        shooter.feed();
        RobotAction notReady = tick(shooter, 0, rpm(4000.0));
        assertEquals("no feed before ready", 0.0, notReady.motor(FEEDER), 0.0);
        tick(shooter, 160, rpm(4000.0));
        assertTrue(shooter.isReady());

        shooter.feed();
        assertTrue(shooter.isFeeding());
        RobotAction start = tick(shooter, 180, rpm(4000.0));
        assertEquals(1.0, start.motor(FEEDER), 0.0);
        assertEquals(1, countEvents(start, "shooter.feed.start"));
        List<Long> on = new ArrayList<>();
        for (long t = 200; t <= 800; t += 20) {
            if (t <= 520) {
                // Holding feed while pulsing must not queue or extend the pulse.
                shooter.feed();
            }
            if (tick(shooter, t, rpm(4000.0)).motor(FEEDER) > 0.0) {
                on.add(t);
            }
        }
        // Started at 180: last on tick 520, off at the first tick >= 530.
        assertEquals(17, on.size());
        assertEquals(520L, (long) on.get(on.size() - 1));
        assertFalse(shooter.isFeeding());
    }

    @Test
    public void feedRefusedDuringDipUntilReReady() {
        FlywheelShooter shooter = new FlywheelShooter();
        shooter.spinUp(4000.0);
        tick(shooter, 0, rpm(4000.0));
        tick(shooter, 160, rpm(4000.0));
        tick(shooter, 180, rpm(3700.0));
        shooter.feed();
        assertEquals(0.0, tick(shooter, 200, rpm(3990.0)).motor(FEEDER), 0.0);
        assertFalse(shooter.isFeeding());
    }

    @Test
    public void missingOrNonFiniteVelocityZeroesBothAndClearsReady() {
        FlywheelShooter shooter = new FlywheelShooter();
        shooter.spinUp(4000.0);
        tick(shooter, 0, rpm(4000.0));
        tick(shooter, 160, rpm(4000.0));
        assertTrue(shooter.isReady());

        RobotAction missing = tick(shooter, 180, Map.of());
        assertEquals(0.0, missing.motor(RIGHT), 0.0);
        assertEquals(0.0, missing.motor(LEFT), 0.0);
        assertFalse(shooter.isReady());
        assertTrue(Double.isNaN(shooter.measuredRpm()));

        RobotAction nan = tick(shooter, 200, Map.of(RIGHT, Double.NaN));
        assertEquals(0.0, nan.motor(RIGHT), 0.0);

        RobotAction back = tick(shooter, 220, rpm(4000.0));
        assertEquals("fresh tick: no derivative kick", 0.728222, back.motor(RIGHT), 1e-6);
    }

    @Test
    public void spinDownZeroesPairAndStopsFeederSameTick() {
        FlywheelShooter shooter = new FlywheelShooter();
        shooter.spinUp(4000.0);
        tick(shooter, 0, rpm(4000.0));
        tick(shooter, 160, rpm(4000.0));
        shooter.feed();
        assertEquals(1.0, tick(shooter, 180, rpm(4000.0)).motor(FEEDER), 0.0);

        shooter.spinDown();
        RobotAction stop = tick(shooter, 200, rpm(4000.0));
        assertEquals(0.0, stop.motor(RIGHT), 0.0);
        assertEquals(0.0, stop.motor(LEFT), 0.0);
        assertEquals(0.0, stop.motor(FEEDER), 0.0);
        assertFalse(shooter.isReady());
        assertFalse(shooter.isFeeding());

        shooter.spinUp(4000.0);
        tick(shooter, 220, rpm(4000.0));
        assertFalse("restart needs a fresh dwell", shooter.isReady());
        tick(shooter, 380, rpm(4000.0));
        assertTrue(shooter.isReady());
    }

    @Test
    public void targetChangeAbove50RpmResetsDwellOnly() {
        FlywheelShooter shooter = new FlywheelShooter();
        shooter.spinUp(4000.0);
        tick(shooter, 0, rpm(4000.0));
        tick(shooter, 160, rpm(4000.0));
        shooter.spinUp(4040.0);
        tick(shooter, 180, rpm(4000.0));
        assertTrue("<=50 RPM change keeps readiness", shooter.isReady());
        shooter.spinUp(4100.0);
        tick(shooter, 200, rpm(4050.0));
        assertFalse(shooter.isReady());
    }

    @Test
    public void invalidSpinUpIsSpinDown() {
        FlywheelShooter shooter = new FlywheelShooter();
        shooter.spinUp(Double.NaN);
        RobotAction action = tick(shooter, 0, rpm(0.0));
        assertEquals(0.0, action.motor(RIGHT), 0.0);
        shooter.spinUp(-100.0);
        assertEquals(0.0, tick(shooter, 20, rpm(0.0)).motor(LEFT), 0.0);
    }

    @Test
    public void liveTuningIsReadOncePerTickAndClearsIntegralAndReady() {
        AtomicReference<ShooterTuning> live = new AtomicReference<>(ShooterTuning.DEFAULTS);
        AtomicInteger reads = new AtomicInteger();
        FlywheelShooter shooter = new FlywheelShooter(() -> {
            reads.incrementAndGet();
            return live.get();
        });
        shooter.spinUp(4000.0);
        RobotAction first = tick(shooter, 0, rpm(3950.0));
        assertEquals(1, reads.get());
        assertEquals(1, countEvents(first, "shooter.tuning"));
        tick(shooter, 20, rpm(3950.0));
        tick(shooter, 180, rpm(3950.0));
        assertTrue(shooter.isReady());
        assertTrue(shooter.integralAccum() > 0.0);
        assertEquals(3, reads.get());

        ShooterTuning d = ShooterTuning.DEFAULTS;
        live.set(new ShooterTuning(d.kS(), d.kV(), 2 * d.kP(), d.kI(), d.kD(),
                d.integralZoneRpm(), d.integralMaxAccum(), 60.0, 300L));
        RobotAction changed = tick(shooter, 200, rpm(3950.0));
        assertEquals(1, countEvents(changed, "shooter.tuning"));
        assertEquals(0.0, shooter.integralAccum(), EPS);
        assertFalse(shooter.isReady());
        // Fresh tick with the new kP only: no mix of old and new gains.
        assertEquals(2 * d.kP() * 50.0 + 0.728222, changed.motor(RIGHT), 1e-6);
        // New 300 ms dwell counts from the reset at 200.
        tick(shooter, 480, rpm(3950.0));
        assertFalse(shooter.isReady());
        tick(shooter, 500, rpm(3950.0));
        assertTrue(shooter.isReady());
        live.set(new ShooterTuning(d.kS(), d.kV(), 2 * d.kP(), d.kI(), d.kD(),
                d.integralZoneRpm(), d.integralMaxAccum(), 40.0, 300L));
        tick(shooter, 520, rpm(3950.0));
        assertFalse("tighter live tolerance applies", shooter.isReady());
    }

    @Test
    public void invalidTuningIsRejectedAndLastValidKept() {
        ShooterTuning d = ShooterTuning.DEFAULTS;
        AtomicReference<ShooterTuning> live = new AtomicReference<>(d);
        FlywheelShooter shooter = new FlywheelShooter(live::get);
        shooter.spinUp(4000.0);
        tick(shooter, 0, rpm(4000.0));

        live.set(new ShooterTuning(Double.NaN, d.kV(), d.kP(), d.kI(), d.kD(),
                d.integralZoneRpm(), d.integralMaxAccum(), d.toleranceRpm(), d.stabilityMs()));
        RobotAction rejected = tick(shooter, 20, rpm(4000.0));
        assertEquals(1, countEvents(rejected, "shooter.tuning.rejected"));
        assertEquals(d, shooter.tuning());
        assertEquals(0.728222, rejected.motor(RIGHT), 1e-6);
        assertEquals(0, countEvents(tick(shooter, 40, rpm(4000.0)), "shooter.tuning.rejected"));

        live.set(new ShooterTuning(d.kS(), d.kV(), d.kP(), d.kI(), d.kD(),
                -1.0, d.integralMaxAccum(), d.toleranceRpm(), d.stabilityMs()));
        tick(shooter, 60, rpm(4000.0));
        live.set(null);
        tick(shooter, 80, rpm(4000.0));
        assertEquals(d, shooter.tuning());
        assertEquals(4, shooter.rejectedTuningCount());

        FlywheelShooter badStart = new FlywheelShooter(() -> null);
        badStart.spinUp(4000.0);
        assertEquals(0.728222, tick(badStart, 0, rpm(4000.0)).motor(RIGHT), 1e-6);
        assertEquals(ShooterTuning.DEFAULTS, badStart.tuning());
    }

    @Test
    public void deterministicReplay() {
        assertEquals(scriptedRun(), scriptedRun());
    }

    @Test
    public void writesBothShooterMotorsEveryTickAndNeverTheTurretEncoder() {
        FlywheelShooter shooter = new FlywheelShooter();
        RobotAction idle = tick(shooter, 0, rpm(0.0));
        assertTrue(idle.motors().containsKey(RIGHT));
        assertTrue(idle.motors().containsKey(LEFT));
        assertEquals(0.0, idle.motor(RIGHT), 0.0);
        assertTrue(idle.servos().isEmpty());
        // shooterLeft velocity (the turret encoder input) never changes shooter control.
        FlywheelShooter a = new FlywheelShooter();
        FlywheelShooter b = new FlywheelShooter();
        a.spinUp(4000.0);
        b.spinUp(4000.0);
        Map<String, Double> withTurret = new HashMap<>(rpm(3900.0));
        withTurret.put(LEFT, 12345.0);
        assertEquals(tick(a, 0, rpm(3900.0)).motor(RIGHT), tick(b, 0, withTurret).motor(RIGHT), 0.0);
    }

    private static List<Double> scriptedRun() {
        List<Double> out = new ArrayList<>();
        AtomicReference<ShooterTuning> live = new AtomicReference<>(ShooterTuning.DEFAULTS);
        FlywheelShooter shooter = new FlywheelShooter(live::get);
        double speed = 0.0;
        for (long t = 0; t <= 4000; t += 20) {
            if (t == 100) {
                shooter.spinUp(4000.0);
            }
            if (t == 2000) {
                ShooterTuning d = ShooterTuning.DEFAULTS;
                live.set(new ShooterTuning(d.kS(), d.kV(), d.kP() * 1.5, d.kI(), d.kD(),
                        d.integralZoneRpm(), d.integralMaxAccum(), d.toleranceRpm(), d.stabilityMs()));
            }
            if (shooter.isReady()) {
                shooter.feed();
            }
            RobotAction action = tick(shooter, t, rpm(speed));
            double power = action.motor(RIGHT);
            // Toy first-order plant, only to exercise a closed loop for replay equality.
            speed += (power * 9600.0 - speed) * 0.02 / 0.5;
            out.add(power);
            out.add(action.motor(FEEDER));
        }
        return out;
    }

    private static int countEvents(RobotAction action, String name) {
        int n = 0;
        for (Event event : action.events()) {
            if (event.name().equals(name)) {
                n++;
            }
        }
        return n;
    }

    private static Map<String, Double> rpm(double wheelRpm) {
        return Map.of(RIGHT, wheelRpm * TICKS_AT_4000 / 4000.0);
    }

    private static RobotAction tick(FlywheelShooter shooter, long t, Map<String, Double> vel) {
        shooter.observe(new RobotState(t, Map.of(), vel, 0.0, new Pose(0.0, 0.0, 0.0), 12.0));
        RobotAction.Builder out = new RobotAction.Builder();
        shooter.update(out);
        return out.build();
    }
}
