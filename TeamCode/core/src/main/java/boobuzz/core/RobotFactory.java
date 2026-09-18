package boobuzz.core;

import boobuzz.core.controller.IController;
import boobuzz.core.controller.teleop.TeleopController;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.hal.IHal;
import boobuzz.core.debug.SubsystemTrace;
import boobuzz.core.logic.EngineRegistry;
import boobuzz.core.logic.IRobotEngine;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.subsystem.intake.PowerIntake;
import boobuzz.core.subsystem.stub.StubShooter;
import boobuzz.core.subsystem.stub.StubTurret;
import boobuzz.core.subsystem.Subsystems;
import boobuzz.core.subsystem.pedro.PedroDrive;

import java.util.List;
import java.util.Objects;

/** Builds the shared engine/controller chain for the robot and simulator. */
public final class RobotFactory {

    private RobotFactory() {}

    public static RobotLoop create(IHal hal, Mechanism mechanism) {
        return create(hal, mechanism, EngineRegistry.CPLX1_NAME, null);
    }

    /** Builds a loop with a fixed per-tick stream instead of the HAL gamepad. */
    public static RobotLoop create(IHal hal, Mechanism mechanism, RequestStream fixedStream) {
        return create(hal, mechanism, EngineRegistry.CPLX1_NAME, fixedStream);
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
        return create(hal, mechanism, engineName, fixedStream, debugTapPort, false);
    }

    /** Builds a loop with optional seam tracing even when the network tap is disabled. */
    public static RobotLoop create(IHal hal, Mechanism mechanism,
                                   String engineName, RequestStream fixedStream,
                                   int debugTapPort, boolean traceEnabled) {
        IController controller = fixedStream == null
                ? new TeleopController(hal)
                : feedback -> new RequestBatch(fixedStream, java.util.Collections.emptyList(),
                        new int[0]);
        return createWithController(hal, mechanism, engineName, controller,
                debugTapPort, traceEnabled);
    }

    /** Builds the selected engine with an explicit controller, used by autonomous entry points. */
    public static RobotLoop createWithController(IHal hal, Mechanism mechanism,
                                                  String engineName, IController controller) {
        return createWithController(hal, mechanism, engineName, controller, 0);
    }

    public static RobotLoop createWithController(IHal hal, Mechanism mechanism,
                                                  String engineName, IController controller,
                                                  int debugTapPort) {
        return createWithController(hal, mechanism, engineName, controller,
                debugTapPort, false);
    }

    /** Builds an explicit-controller loop with optional trace recording for bagging. */
    public static RobotLoop createWithController(IHal hal, Mechanism mechanism,
                                                  String engineName, IController controller,
                                                  int debugTapPort, boolean traceEnabled) {
        Objects.requireNonNull(hal, "hal");
        Objects.requireNonNull(mechanism, "mechanism");
        Objects.requireNonNull(engineName, "engineName");
        Objects.requireNonNull(controller, "controller");
        Subsystems baseSubsystems = new Subsystems(
                new PedroDrive(mechanism), new StubShooter(), new PowerIntake(), new StubTurret());
        SubsystemTrace trace = traceEnabled || debugTapPort != 0 ? new SubsystemTrace() : null;
        Subsystems subsystems = trace == null
                ? baseSubsystems : SubsystemTrace.wrap(baseSubsystems, trace);
        List<IRobotEngine> engines = EngineRegistry.create(subsystems);
        IRobotEngine engine = EngineRegistry.find(
                EngineRegistry.bind(engines), EngineRegistry.indexForName(engineName));
        if (engine == null) {
            throw new IllegalArgumentException("engine is not available: " + engineName);
        }
        RobotLoop loop = new RobotLoop(hal, engines, engine,
                controller, debugTapPort);
        loop.setSubsystemTrace(trace);
        return loop;
    }
}
