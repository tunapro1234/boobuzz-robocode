package boobuzz.core;

import boobuzz.core.control.Drive;
import boobuzz.core.control.Intent;
import boobuzz.core.control.Request;
import boobuzz.core.control.RequestStatus;
import boobuzz.core.control.RequestType;
import boobuzz.core.engine.C1DriveEngine;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;
import boobuzz.core.mechanism.Mechanism;

import com.pedropathing.math.Pose;

import org.junit.Before;
import org.junit.Test;

import java.io.InputStream;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class C1DriveEngineTest {

    private static final double EPS = 1e-9;

    private Mechanism mechanism;
    private C1DriveEngine engine;

    @Before
    public void setUp() {
        try (InputStream in = getClass().getResourceAsStream("/mechanism-test.yaml")) {
            mechanism = Mechanism.load(in, "mechanism-test.yaml");
        } catch (Exception e) {
            throw new AssertionError(e);
        }
        engine = new C1DriveEngine(mechanism);
    }

    @Test
    public void koseleriDogruEslestirir() {
        assertEquals("fl", engine.frontLeftMotor());
        assertEquals("fr", engine.frontRightMotor());
        assertEquals("bl", engine.backLeftMotor());
        assertEquals("br", engine.backRightMotor());
    }

    @Test
    public void ileriKomutuDortMotoruIleriSurer() {
        RobotAction a = engine.act(Intent.of(new Drive.Manual(1, 0, 0)));
        assertEquals(1.0, a.motor("fl"), EPS);
        assertEquals(1.0, a.motor("fr"), EPS);
        assertEquals(1.0, a.motor("bl"), EPS);
        assertEquals(1.0, a.motor("br"), EPS);
    }

    @Test
    public void geriKomutuDortMotoruGeriSurer() {
        RobotAction a = engine.act(Intent.of(new Drive.Manual(-1, 0, 0)));
        assertEquals(-1.0, a.motor("fl"), EPS);
        assertEquals(-1.0, a.motor("fr"), EPS);
        assertEquals(-1.0, a.motor("bl"), EPS);
        assertEquals(-1.0, a.motor("br"), EPS);
    }

    @Test
    public void ccwDonusIsaretleriDogru() {
        // omega CCW(+): sol taraf geri, sag taraf ileri.
        RobotAction a = engine.act(Intent.of(new Drive.Manual(0, 0, 1)));
        assertEquals(-1.0, a.motor("fl"), EPS);
        assertEquals(+1.0, a.motor("fr"), EPS);
        assertEquals(-1.0, a.motor("bl"), EPS);
        assertEquals(+1.0, a.motor("br"), EPS);
    }

    @Test
    public void solaKaymaXDeseniUretir() {
        // vy SOL(+): fl geri, fr ileri, bl ileri, br geri.
        RobotAction a = engine.act(Intent.of(new Drive.Manual(0, 1, 0)));
        assertEquals(-1.0, a.motor("fl"), EPS);
        assertEquals(+1.0, a.motor("fr"), EPS);
        assertEquals(+1.0, a.motor("bl"), EPS);
        assertEquals(-1.0, a.motor("br"), EPS);
    }

    @Test
    public void doygunlukYonuKorur() {
        // Ileri + kayma + donus hepsi 1 iken toplam 3'e cikar; oranlar korunmali.
        RobotAction a = engine.act(Intent.of(new Drive.Manual(1, 1, 1)));
        double peak = Math.max(Math.max(Math.abs(a.motor("fl")), Math.abs(a.motor("fr"))),
                Math.max(Math.abs(a.motor("bl")), Math.abs(a.motor("br"))));
        assertEquals(1.0, peak, EPS);
        // ham: fl=1-1-1=-1  fr=1+1+1=3  bl=1+1-1=1  br=1-1+1=1  -> hepsi /3
        assertEquals(-1.0 / 3.0, a.motor("fl"), EPS);
        assertEquals(1.0, a.motor("fr"), EPS);
        assertEquals(1.0 / 3.0, a.motor("bl"), EPS);
        assertEquals(1.0 / 3.0, a.motor("br"), EPS);
    }

    @Test
    public void holdSifirGucVerir() {
        RobotAction a = engine.act(Intent.idle());
        for (String m : mechanism.motorNames()) {
            assertEquals(0.0, a.motor(m), EPS);
        }
    }

    @Test
    public void c1IstekleriReddeder() {
        Intent intent = new Intent(Drive.HOLD,
                List.of(Request.of(7, RequestType.SHOOT, 3)), new int[0]);
        engine.act(intent);
        // Durum bir sonraki sense()'te doner (bir tick gecikmeli, anayasa kural 4).
        var statuses = engine.sense(100, emptyState()).statuses();
        assertEquals(1, statuses.size());
        assertEquals(7, statuses.get(0).id());
        assertEquals(RequestStatus.State.REJECTED, statuses.get(0).state());
    }

    @Test
    public void senseHamDurumuGecirir() {
        RobotState s = new RobotState(1234, Map.of(), Map.of(), 0.5,
                new Pose(3, 4, 0.25), 12.1);
        var fb = engine.sense(1234, s);
        assertEquals(1234, fb.t());
        assertEquals(3.0, fb.world().pose().x(), EPS);
        assertEquals(4.0, fb.world().pose().y(), EPS);
        assertEquals(0.5, fb.world().yaw(), EPS);
        assertEquals(12.1, fb.world().voltage(), EPS);
        assertTrue(fb.statuses().isEmpty());
    }

    private static RobotState emptyState() {
        return new RobotState(100, Map.of(), Map.of(), 0, new Pose(0, 0, 0), 12.6);
    }
}
