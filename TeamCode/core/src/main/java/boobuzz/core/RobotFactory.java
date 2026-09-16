package boobuzz.core;

import boobuzz.core.controller.IController;
import boobuzz.core.controller.teleop.TeleopController;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.hal.IHal;
import boobuzz.core.debug.SubsystemTrace;
import boobuzz.core.logic.cplx1.CplxEngine1;
import boobuzz.core.logic.direct.DirectEngine;
import boobuzz.core.logic.IRobotEngine;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.subsystem.stub.StubIntake;
import boobuzz.core.subsystem.stub.StubShooter;
import boobuzz.core.subsystem.stub.StubTurret;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.pedro.PedroDrive;

import java.util.Objects;

/** Builds the shared engine/controller chain for the robot and simulator. */
public final class RobotFactory {

    private RobotFactory() {}

    public static RobotLoop create(IHal hal, Mechanism mechanism) {
        return create(hal, mechanism, "cplx1", null);
    }

    /** Builds a loop with a fixed per-tick stream instead of the HAL gamepad. */
    public static RobotLoop create(IHal hal, Mechanism mechanism, RequestStream fixedStream) {
        return create(hal, mechanism, "cplx1", fixedStream);
    }

    public static RobotLoop create(IHal hal, Mechanism mechanism, String engineName) {
        return create(hal, mechanism, engineName, null);
    }

    /** Builds the selected engine over one fixed-order subsystem set. */
    public static RobotLoop create(IHal hal, Mechanism mechanism,
                                   String engineName, RequestStream fixedStream) {
        return create(hal, mechanism, engineName, fixedStream, 0);
    }

    public static RobotLoop create(IHal hal, Mechanism mechanism,
                                   String engineName, RequestStream fixedStream,
                                   int debugTapPort) {
        IController controller = fixedStream == null
                ? new TeleopController(hal)
                : feedback -> new RequestBatch(fixedStream, java.util.List.of(), new int[0]);
        return createWithController(hal, mechanism, engineName, controller, debugTapPort);
    }

    /** Builds the selected engine with an explicit controller, used by autonomous entry points. */
    public static RobotLoop createWithController(IHal hal, Mechanism mechanism,
                                                  String engineName, IController controller) {
        return createWithController(hal, mechanism, engineName, controller, 0);
    }

    public static RobotLoop createWithController(IHal hal, Mechanism mechanism,
                                                  String engineName, IController controller,
                                                  int debugTapPort) {
        Objects.requireNonNull(hal, "hal");
        Objects.requireNonNull(mechanism, "mechanism");
        Objects.requireNonNull(engineName, "engineName");
        Objects.requireNonNull(controller, "controller");
        Subsystems baseSubsystems = new Subsystems(
                new PedroDrive(mechanism), new StubShooter(), new StubIntake(), new StubTurret());
        SubsystemTrace trace = debugTapPort == 0 ? null : new SubsystemTrace();
        Subsystems subsystems = trace == null
                ? baseSubsystems : SubsystemTrace.wrap(baseSubsystems, trace);
        DirectEngine direct = new DirectEngine(subsystems);
        CplxEngine1 cplx1 = new CplxEngine1(subsystems);
        IRobotEngine engine = switch (engineName) {
            case "cplx1", "cplx_engine_1" -> cplx1;
            case "direct" -> direct;
            default -> throw new IllegalArgumentException(
                    "unknown engine: " + engineName + " (expected direct or cplx1)");
        };
        RobotLoop loop = new RobotLoop(hal, java.util.List.of(direct, cplx1), engine,
                controller, debugTapPort);
        loop.setSubsystemTrace(trace);
        return loop;
    }
}
