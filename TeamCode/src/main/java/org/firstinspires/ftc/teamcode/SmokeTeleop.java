package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Pedro 3.0 stack'inin ayakta oldugunu dogrulayan minimal teleop.
 * Robot hazir olunca gercek teleop bunun yerini alacak.
 */
@TeleOp(name = "Smoke Teleop", group = "0 Test")
public class SmokeTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        Hardware robot = new Hardware(hardwareMap, new Pose(0, 0, 0));

        telemetry.addLine("Pedro 3.0 hazir. Start'a bas.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            robot.follower.update();
            robot.follower.manual(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x);

            Pose pose = robot.follower.pose();
            telemetry.addData("x", "%.1f", pose.x());
            telemetry.addData("y", "%.1f", pose.y());
            telemetry.addData("heading", "%.1f", Math.toDegrees(pose.heading()));
            telemetry.update();
        }
    }
}
