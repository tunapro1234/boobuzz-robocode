package org.firstinspires.ftc.teamcode.opmode;

import boobuzz.core.RobotFactory;
import boobuzz.core.RobotLoop;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.MechanismLoader;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hal.RealHal;

/** Main TeleOp running the same core bytecode as the simulator on the real HAL. */
@TeleOp(name = "Teleop Main", group = "1 Main")
public class TeleopMain extends LinearOpMode {

    @Override
    public void runOpMode() {
        Mechanism mechanism = MechanismLoader.loadDefault();
        RealHal hal = new RealHal(hardwareMap, gamepad1, mechanism, new Pose(0, 0, 0));
        RobotLoop robot = RobotFactory.create(hal, mechanism);

        telemetry.addLine("RealHal + shared core ready. Press Start.");
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
