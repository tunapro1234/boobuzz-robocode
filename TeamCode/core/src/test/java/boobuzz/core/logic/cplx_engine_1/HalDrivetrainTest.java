package boobuzz.core.logic.cplx_engine_1;

import boobuzz.core.hal.RobotAction;

import com.pedropathing.drivetrain.DrivePowers;

import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class HalDrivetrainTest {

    private static final double EPS = 1e-9;
    private HalDrivetrain drivetrain;

    @Before
    public void setUp() {
        drivetrain = new HalDrivetrain(new String[] {"fl", "fr", "bl", "br"});
    }

    @Test
    public void drivePowersMotorAdlariylaSonEylemeDonusur() {
        drivetrain.drive(new DrivePowers(0.2, -0.3, 0.1), false);
        RobotAction action = drivetrain.lastAction();

        assertEquals(4, action.motors().size());
        assertEquals(0.4, action.motor("fl"), EPS);
        assertEquals(0.0, action.motor("fr"), EPS);
        assertEquals(-0.2, action.motor("bl"), EPS);
        assertEquals(0.6, action.motor("br"), EPS);
    }

    @Test
    public void tekerlekGucleriniEksiBirArtiBirAraliginaKirpar() {
        drivetrain.drive(new DrivePowers(2.0, 0.0, 0.0), false);
        RobotAction positive = drivetrain.lastAction();
        assertEquals(1.0, positive.motor("fl"), EPS);
        assertEquals(1.0, positive.motor("fr"), EPS);

        drivetrain.drive(new DrivePowers(-2.0, 0.0, 0.0), false);
        RobotAction negative = drivetrain.lastAction();
        assertEquals(-1.0, negative.motor("bl"), EPS);
        assertEquals(-1.0, negative.motor("br"), EPS);
    }

    @Test
    public void doygunTabandaTersDeltaOlsaBileOlceklemez() {
        double scale = drivetrain.maxScaling(
                new DrivePowers(2.0, 0.0, 0.0),
                new DrivePowers(-1.0, 0.0, 0.0));

        assertEquals(0.0, scale, EPS);
    }

    @Test
    public void sinirIcindeTersIsaretliDeltaninTamaminiUygular() {
        double scale = drivetrain.maxScaling(
                new DrivePowers(0.8, 0.0, 0.0),
                new DrivePowers(-0.2, 0.0, 0.0));

        assertEquals(1.0, scale, EPS);
    }

    @Test
    public void sifirDeltaTamOlcekDondurur() {
        double scale = drivetrain.maxScaling(
                new DrivePowers(0.4, 0.0, 0.0), DrivePowers.zero());

        assertEquals(1.0, scale, EPS);
    }
}
