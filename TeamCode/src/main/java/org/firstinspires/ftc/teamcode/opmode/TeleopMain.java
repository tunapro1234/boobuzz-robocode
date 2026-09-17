package org.firstinspires.ftc.teamcode.opmode;

import boobuzz.core.RobotFactory;
import boobuzz.core.RobotLoop;
import boobuzz.core.controller.IController;
import boobuzz.core.controller.socket.SocketController;
import boobuzz.core.controller.teleop.TeleopController;
import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.RobotConstants;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;

import org.firstinspires.ftc.teamcode.hal.RealHal;

/** Main TeleOp running the same core bytecode as the simulator on the real HAL. */
@TeleOp(name = "Teleop Main", group = "1 Main")
public class TeleopMain extends LinearOpMode {

    @Override
    public void runOpMode() {
        Mechanism mechanism = RobotConstants.mechanism();
        RealHal hal = new RealHal(hardwareMap, gamepad1, mechanism, new Pose(0, 0, 0));
        SocketController socketController = null;
        IController controller;
        if ("socket".equals(RobotConstants.DEFAULT_CONTROLLER)) {
            try {
                socketController = new SocketController();
                controller = socketController;
            } catch (IOException e) {
                telemetry.addData("controller", "socket unavailable: %s", e.getMessage());
                telemetry.update();
                return;
            }
        } else {
            controller = new TeleopController(hal);
        }
        RobotLoop robot = null;
        try {
            // Keep controller and loop construction in the same cleanup scope:
            // a factory failure must not leak the socket controller workers.
            robot = RobotFactory.createWithController(
                    hal, mechanism, "cplx1", controller, RobotConstants.REAL_DEBUG_TAP_PORT);

            telemetry.addLine("RealHal + shared core ready. Press Start.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                robot.tick();
                telemetry.addData("engine", robot.engine().name());
                telemetry.addData("ticks", robot.ticks());
                telemetry.update();
                // Yield to the FTC scheduler so the watchdog and hardware threads run.
                idle();
            }
        } finally {
            if (robot != null) {
                robot.close();
            }
            if (socketController != null) {
                socketController.close();
            }
        }
    }
}
