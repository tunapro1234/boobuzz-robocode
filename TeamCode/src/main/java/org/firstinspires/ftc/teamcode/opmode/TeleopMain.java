package org.firstinspires.ftc.teamcode.opmode;

import boobuzz.core.RobotFactory;
import boobuzz.core.RobotLoop;
import boobuzz.core.hal.Mechanism;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hal.RealHal;

/** Gercek HAL uzerinde sim ile ayni core bytecode'unu calistiran ana TeleOp. */
@TeleOp(name = "Teleop Main", group = "1 Main")
public class TeleopMain extends LinearOpMode {

    @Override
    public void runOpMode() {
        Mechanism mechanism = Mechanism.loadDefault();
        RealHal hal = new RealHal(hardwareMap, gamepad1, mechanism, new Pose(0, 0, 0));
        RobotLoop robot = RobotFactory.create(hal, mechanism);

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
