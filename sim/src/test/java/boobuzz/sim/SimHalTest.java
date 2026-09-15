package boobuzz.sim;

import boobuzz.core.RobotLoop;
import boobuzz.core.contract.Drive;
import boobuzz.core.controller.GamepadController;
import boobuzz.core.contract.Intent;
import boobuzz.core.logic.cplx_engine_1.CplxEngine1;
import boobuzz.core.hal.GamepadState;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.MechanismLoader;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/**
 * Python olmadan protokol dogrulamasi. Gercek entegrasyon
 * re-cock-nize sunucusuyla ayrica kosulur.
 */
public class SimHalTest {

    private static Mechanism mechanism() throws Exception {
        Path p = Path.of("..", "mechanism.yaml").toAbsolutePath().normalize();
        assertTrue("mechanism.yaml bulunamadi: " + p, Files.exists(p));
        return MechanismLoader.load(p);
    }

    private static SimHal connect(FakeSimServer server, Mechanism m, int dtMs) throws Exception {
        return new SimHal(m, "127.0.0.1", server.port(), dtMs, 0L, new Pose(0, 0, 0), 2000);
    }

    @Test
    public void elSikismaVeAdDogrulamasi() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m.motorNames());
             SimHal hal = connect(server, m, 20)) {
            // Baslangic state 'ready' icinde geldi: saat hic ilerlememis olmali.
            assertEquals(0L, hal.now());
            assertEquals(0.0, hal.read().pinpoint().x(), 1e-9);
            assertEquals(0.0, hal.read().pinpoint().y(), 1e-9);
            RobotState s = hal.read();
            assertEquals(12.6, s.voltage(), 1e-9);
            assertNotNull(s.pinpoint());
        }
    }

    @Test
    public void readyStateTasimazsaCoker() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m.motorNames())) {
            server.setOmitReadyState(true);
            SimProtocolException e = assertThrows(SimProtocolException.class,
                    () -> connect(server, m, 20).close());
            assertTrue(e.getMessage().contains("state"));
        }
    }

    @Test
    public void adUyusmazligindaCoker() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(List.of("fl", "fr", "bl"))) {
            assertThrows(Mechanism.MechanismException.class, () -> connect(server, m, 20).close());
        }
    }

    @Test
    public void besYuzAdimHatasizKosar() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m.motorNames());
             SimHal hal = connect(server, m, 20)) {

            RobotLoop loop = new RobotLoop(hal, new CplxEngine1(m),
                    fb -> Intent.of(new Drive.Manual(1, 0, 0)));

            for (int i = 0; i < 500; i++) {
                loop.tick();
            }
            assertEquals(500, loop.ticks());
            assertEquals(500L * 20, hal.now());   // hazirlik adimi yok
            // Ileri surulduyse poz +x yonunde ilerlemis olmali.
            assertTrue("robot ileri gitmedi: " + hal.read().pinpoint().x(),
                    hal.read().pinpoint().x() > 1.0);
        }
    }

    @Test
    public void saatHaldenGelirDuvarSaatindenDegil() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m.motorNames());
             SimHal hal = connect(server, m, 50)) {
            RobotLoop loop = new RobotLoop(hal, new CplxEngine1(m), fb -> Intent.idle());
            long wallStart = System.nanoTime();
            for (int i = 0; i < 200; i++) {
                loop.tick();
            }
            long wallMs = (System.nanoTime() - wallStart) / 1_000_000;
            assertEquals(200L * 50, hal.now());        // 200 x 50 ms sim zamani
            assertTrue("sim gercek zamandan hizli kosmali, gercek=" + wallMs + "ms",
                    wallMs < 10_000);
        }
    }

    @Test
    public void gamepadSunucudanGelirVeNiyeteDonusur() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m.motorNames())) {
            server.setGamepadJson("{\"lx\":0,\"ly\":-1.0,\"rx\":0,\"ry\":0,\"a\":true,"
                    + "\"b\":false,\"x\":false,\"y\":false,\"lb\":false,\"rb\":false,"
                    + "\"lt\":0,\"rt\":0.5,\"dpad\":\"up\"}");
            try (SimHal hal = connect(server, m, 20)) {
                GamepadState g = hal.get();
                assertEquals(-1.0, g.ly(), 1e-9);
                assertTrue(g.a());
                assertEquals(0.5, g.rt(), 1e-9);
                assertEquals(GamepadState.Dpad.UP, g.dpad());

                Drive.Manual d = (Drive.Manual) new GamepadController(hal).decide(null).drive();
                assertEquals(1.0, d.vx(), 1e-9);
            }
        }
    }

    @Test
    public void truthCoreDisindaKalir() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m.motorNames());
             SimHal hal = connect(server, m, 20)) {
            hal.write(RobotAction.zero());
            // truth SimHal'de var (viewer/test icin) ama RobotState'te yok.
            assertNotNull(hal.truth());
            for (var field : RobotState.class.getRecordComponents()) {
                assertTrue("RobotState'e truth sizmis: " + field.getName(),
                        !field.getName().toLowerCase().contains("truth"));
            }
        }
    }
}
