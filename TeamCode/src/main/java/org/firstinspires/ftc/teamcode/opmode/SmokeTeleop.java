package org.firstinspires.ftc.teamcode.opmode;

import boobuzz.core.RobotLoop;
import boobuzz.core.controller.GamepadController;
import boobuzz.core.logic.engine.PedroDriveEngine;
import boobuzz.core.mechanism.Mechanism;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hal.RealHal;

/**
 * Pedro 3.0 stack'inin ayakta oldugunu dogrulayan minimal teleop.
 * Robot hazir olunca gercek teleop bunun yerini alacak.
 */
@TeleOp(name = "Smoke Teleop", group = "0 Test")
public class SmokeTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        Mechanism mechanism = Mechanism.loadDefault();
        RealHal hal = new RealHal(hardwareMap, gamepad1, mechanism, new Pose(0, 0, 0));
        RobotLoop robot = new RobotLoop(
                hal, new PedroDriveEngine(mechanism), new GamepadController(hal));

        telemetry.addLine("RealHal + ortak core hazir. Start'a bas.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            robot.tick();
            telemetry.addData("engine", robot.engine().name());
            telemetry.addData("ticks", robot.ticks());
            telemetry.update();
        }
    }
}
