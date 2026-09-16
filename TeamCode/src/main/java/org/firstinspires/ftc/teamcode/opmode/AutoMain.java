package org.firstinspires.ftc.teamcode.opmode;

import boobuzz.core.RobotFactory;
import boobuzz.core.RobotLoop;
import boobuzz.core.controller.auto.AutoController;
import boobuzz.core.controller.auto.AutoSequence;
import boobuzz.core.controller.autos.AutoRegistry;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.RobotConstants;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hal.RealHal;

/** Shared FTC shell for the registry-backed autonomous routines. */
public abstract class AutoMain extends LinearOpMode {

    /** Registry key for this opmode's autonomous routine. */
    protected abstract String routineName();

    @Override
    public final void runOpMode() {
        Mechanism mechanism = RobotConstants.mechanism();
        AutoSequence sequence = AutoRegistry.build(routineName());
        AutoController controller = new AutoController(sequence);
        RealHal hal = new RealHal(hardwareMap, gamepad1, mechanism, sequence.startPose());
        RobotLoop robot = RobotFactory.createWithController(
                hal, mechanism, "cplx_engine_1", controller);

        telemetry.addData("auto", routineName());
        telemetry.addData("steps", sequence.size());
        telemetry.addLine("Ready. Press Start.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !controller.isFinished()) {
            robot.tick();
            telemetry.addData("step", sequence.index() + "/" + sequence.size());
            telemetry.addData("command", sequence.currentName());
            telemetry.update();
        }

        if (controller.isFailed()) {
            telemetry.addData("auto failure", controller.failureNote());
        } else {
            telemetry.addLine("Auto complete");
        }
        Pose pose = hal.read().pinpoint();
        telemetry.addData("final pose", "(%.1f, %.1f, %.1f deg)",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.update();
    }
}
