package org.firstinspires.ftc.teamcode.hal;

import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.RobotConstants;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Collections;

/** Finds only FTC devices and configures them from {@link RobotConstants}. */
public final class Hardware {

    final Map<String, DcMotorEx> motors;
    final Map<String, Servo> servos;
    final GoBildaPinpointDriver pinpoint;
    final List<VoltageSensor> voltageSensors;

    public Hardware(HardwareMap hardwareMap, Mechanism mechanism, Pose startPose) {
        Map<String, DcMotorEx> foundMotors = new LinkedHashMap<>();
        for (String name : mechanism.motorNames()) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
            Mechanism.Motor config = mechanism.motor(name);
            if ("wheel".equals(config.drives())) {
                // In the robot frame +left is the left side; the center threshold (0) counts as left.
                motor.setDirection(config.left() >= 0.0
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            foundMotors.put(name, motor);
        }
        motors = Collections.unmodifiableMap(new LinkedHashMap<>(foundMotors));

        Map<String, Servo> foundServos = new LinkedHashMap<>();
        for (String name : mechanism.servoNames()) {
            foundServos.put(name, hardwareMap.get(Servo.class, name));
        }
        servos = Collections.unmodifiableMap(new LinkedHashMap<>(foundServos));

        Mechanism.Pinpoint config = mechanism.pinpoint();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(config.xPodOffsetMm(), config.yPodOffsetMm(), DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.valueOf(
                config.podType()));
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.valueOf(config.xPodDirection()),
                GoBildaPinpointDriver.EncoderDirection.valueOf(config.yPodDirection()));
        pinpoint.recalibrateIMU();
        Pose pose = startPose == null ? new Pose(0.0, 0.0, 0.0) : startPose;
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.x(), pose.y(),
                AngleUnit.RADIANS, pose.heading()));

        voltageSensors = Collections.unmodifiableList(
                new java.util.ArrayList<>(hardwareMap.getAll(VoltageSensor.class)));
        if (voltageSensors.isEmpty()) {
            throw new IllegalStateException("FTC HardwareMap contains no voltage sensor");
        }
    }
}
