package boobuzz.sim;

import boobuzz.core.RobotLoop;
import boobuzz.core.controller.teleop.TeleopController;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.logic.cplx1.CplxEngine1;
import boobuzz.core.contract.GamepadState;
import boobuzz.core.contract.ActionValidator;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.subsystem.stub.StubIntake;
import boobuzz.core.subsystem.stub.StubShooter;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.pedro.PedroDrive;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.List;
import java.util.Map;
import java.nio.charset.StandardCharsets;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

/**
 * Protocol validation without Python. Real integration runs separately
 * against the re-cock-nize server.
 */
public class SimHalTest {

    private static Mechanism mechanism() {
        return Mechanism.DEFAULT;
    }

    private static Subsystems subsystems(Mechanism mechanism) {
        return new Subsystems(new PedroDrive(mechanism),
                new StubShooter(), new StubIntake());
    }

    private static SimHal connect(FakeSimServer server, Mechanism m, int dtMs) throws Exception {
        return new SimHal(m, "127.0.0.1", server.port(), dtMs, 0L, new Pose(0, 0, 0), 2000);
    }

    @Test
    public void handshakeAndNameValidation() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m);
             SimHal hal = connect(server, m, 20)) {
            // Initial state arrived in 'ready': the clock must not have advanced.
            assertEquals(0L, hal.now());
            assertEquals(0.0, hal.read().pinpoint().x(), 1e-9);
            assertEquals(0.0, hal.read().pinpoint().y(), 1e-9);
            RobotState s = hal.read();
            assertEquals(12.6, s.voltage(), 1e-9);
            assertNotNull(s.pinpoint());
        }
    }

    @Test
    public void missingReadyStateFails() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m)) {
            server.setOmitReadyState(true);
            SimProtocolException e = assertThrows(SimProtocolException.class,
                    () -> connect(server, m, 20).close());
            assertTrue(e.getMessage().contains("state"));
        }
    }

    @Test
    public void readTimeoutFailsWithClearSimulatorError() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m)) {
            server.setReadyDelayMs(250);
            SimProtocolException e = assertThrows(SimProtocolException.class,
                    () -> new SimHal(m, "127.0.0.1", server.port(), 20, 0L,
                            new Pose(0, 0, 0), 2000, 25));
            assertTrue(e.getMessage().contains("timed out waiting for simulator response"));
            assertTrue(e.getMessage().contains("25 ms"));
        }
    }

    @Test
    public void nameMismatchFails() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(
                List.of("fl", "fr", "bl"), m.servoNames(), m.encoderNames(), 2)) {
            assertThrows(Mechanism.MechanismException.class, () -> connect(server, m, 20).close());
        }
    }

    @Test
    public void rejectsProto1ForMechanismProfile() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m.motorNames(), m.servoNames(),
                m.encoderNames(), 1)) {
            SimProtocolException e = assertThrows(SimProtocolException.class,
                    () -> connect(server, m, 20).close());
            assertTrue(e.getMessage().contains("protocol version mismatch"));
        }
    }

    @Test
    public void omittedServoIsNotZeroFilledAndExplicitZeroIsSent() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m);
             SimHal hal = connect(server, m, 20)) {
            hal.write(new RobotAction(Map.of(), Map.of(
                    "hood_left", 1.0, "hood_right", 0.0)));
            hal.write(RobotAction.zero());
            List<Map<String, Double>> frames = server.servoFrames();
            assertEquals(2, frames.size());
            assertEquals(Map.of("hood_left", 1.0, "hood_right", 0.0), frames.get(0));
            assertTrue(frames.get(1).isEmpty());
        }
    }

    @Test
    public void invalidFrameZerosPowerBeforeFailing() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m);
             SimHal hal = connect(server, m, 20)) {
            hal.write(new RobotAction(Map.of("fl", 0.25, "fr", 0.25,
                    "bl", 0.25, "br", 0.25), Map.of()));
            assertThrows(ActionValidator.ValidationException.class,
                    () -> hal.write(new RobotAction(Map.of("fl", Double.NaN), Map.of())));
            List<Map<String, Double>> frames = server.motorFrames();
            assertEquals(2, frames.size());
            assertEquals(0.0, frames.get(1).get("fl"), 0.0);
            assertEquals(0.0, frames.get(1).get("turret_servo2"), 0.0);
        }
    }

    @Test
    public void protocolV2FixturesKeepSparseTwoMapShape() throws Exception {
        Map<String, Object> ready = fixture("ready.json");
        assertEquals(2.0, Json.num(ready, "proto", -1), 0.0);
        assertEquals(10, Json.strings(ready, "motors").size());
        assertEquals(2, Json.strings(ready, "servos").size());
        assertEquals(8, Json.obj(Json.obj(ready, "state"), "enc").size());

        Map<String, Object> hold = fixture("step-hold.json");
        assertTrue(Json.obj(hold, "servos").isEmpty());
        Map<String, Object> explicitZero = fixture("step-explicit-zero.json");
        assertEquals(0.0, Json.num(Json.obj(explicitZero, "servos"), "hood_right", -1), 0.0);
    }

    private static Map<String, Object> fixture(String name) throws Exception {
        try (var stream = SimHalTest.class.getResourceAsStream("/protocol-v2/" + name)) {
            if (stream == null) throw new AssertionError("missing protocol fixture: " + name);
            return Json.parseObject(new String(stream.readAllBytes(), StandardCharsets.UTF_8));
        }
    }

    @Test
    public void runsFiveHundredStepsWithoutError() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m);
             SimHal hal = connect(server, m, 20)) {

            RobotLoop loop = new RobotLoop(hal, new CplxEngine1(subsystems(m)),
                    fb -> new RequestBatch(RequestStream.manual(1, 0, 0), List.of()));

            for (int i = 0; i < 500; i++) {
                loop.tick();
            }
            assertEquals(500, loop.ticks());
            assertEquals(500L * 20, hal.now());   // no setup step counted
            // Driving forward must advance the pose along +x.
            assertTrue("robot did not move forward: " + hal.read().pinpoint().x(),
                    hal.read().pinpoint().x() > 1.0);
        }
    }

    @Test
    public void gamepadArrivesFromServerAndBecomesIntent() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m)) {
            server.setGamepadJson("{\"lx\":0,\"ly\":-1.0,\"rx\":0,\"ry\":0,\"a\":true,"
                    + "\"b\":false,\"x\":false,\"y\":false,\"lb\":false,\"rb\":false,"
                    + "\"lt\":0,\"rt\":0.5,\"dpad\":\"up\"}");
            try (SimHal hal = connect(server, m, 20)) {
                GamepadState g = hal.get();
                assertEquals(-1.0, g.ly(), 1e-9);
                assertTrue(g.a());
                assertEquals(0.5, g.rt(), 1e-9);
                assertEquals(GamepadState.Dpad.UP, g.dpad());

                RequestStream d = new TeleopController(hal).decide(null).stream();
                assertEquals(1.0, d.vx(), 1e-9);
            }
        }
    }

    @Test
    public void truthStaysOutsideCore() throws Exception {
        Mechanism m = mechanism();
        try (FakeSimServer server = new FakeSimServer(m);
             SimHal hal = connect(server, m, 20)) {
            hal.write(RobotAction.zero());
            // truth exists in SimHal (viewer/tests) but not in RobotState.
            assertNotNull(hal.truth());
            for (var field : RobotState.class.getRecordComponents()) {
                assertTrue("truth leaked into RobotState: " + field.getName(),
                        !field.getName().toLowerCase().contains("truth"));
            }
        }
    }
}
