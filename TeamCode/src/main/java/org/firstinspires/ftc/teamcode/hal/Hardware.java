package org.firstinspires.ftc.teamcode.hal;

import boobuzz.core.hal.Mechanism;
import boobuzz.core.hal.RobotConstants;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/** Finds each FTC device once and configures it from {@link RobotConstants}. */
public final class Hardware {

    final Map<String, DcMotorEx> motors;
    final Map<String, DcMotorEx> encoders;
    final Map<String, CRServo> crServos;
    final Map<String, Servo> servos;
    final GoBildaPinpointDriver pinpoint;
    final List<VoltageSensor> voltageSensors;

    public Hardware(HardwareMap hardwareMap, Mechanism mechanism, Pose startPose) {
        Map<String, DcMotorEx> foundMotors = new LinkedHashMap<>();
        Map<String, DcMotorEx> foundEncoders = new LinkedHashMap<>();
        Set<String> resetNames = Set.of(
                RobotConstants.SHOOTER_RIGHT_MOTOR_NAME,
                RobotConstants.SHOOTER_LEFT_MOTOR_NAME,
                RobotConstants.FEEDER_MOTOR_NAME);

        // Wheel keys are stable protocol names; HardwareMap uses the archive port names.
        for (String name : mechanism.wheelMotorNames()) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, wheelHardwareName(name));
            Mechanism.Motor config = mechanism.motor(name);
            // +left is the robot-left side; preserve the established wheel inversion.
            motor.setDirection(config.left() >= 0.0
                    ? DcMotorSimple.Direction.REVERSE
                    : DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            foundMotors.put(name, motor);
            addEncoderAlias(foundEncoders, encoderNameForWheel(name), motor);
        }

        for (RobotConstants.DcDevice device : mechanism.dcDevices()) {
            requireFreshName(foundMotors, device.name());
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, device.name());
            configureDcMotor(motor, device);
            if (resetNames.contains(device.name())) {
                // Central init owns each shared encoder reset exactly once, before enable.
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            foundMotors.put(device.name(), motor);
            addEncoderAlias(foundEncoders, device.name(), motor);
        }
        motors = immutable(foundMotors);
        encoders = immutable(foundEncoders);

        Map<String, CRServo> foundCrServos = new LinkedHashMap<>();
        for (RobotConstants.CrServo device : mechanism.crServos()) {
            if (foundCrServos.containsKey(device.name())) {
                throw new IllegalStateException("duplicate CR servo declaration: " + device.name());
            }
            CRServo servo = hardwareMap.get(CRServo.class, device.name());
            servo.setDirection(DcMotorSimple.Direction.valueOf(device.direction()));
            foundCrServos.put(device.name(), servo);
        }
        crServos = immutable(foundCrServos);

        Map<String, Servo> foundServos = new LinkedHashMap<>();
        for (RobotConstants.PosServo device : mechanism.positionalServos()) {
            if (foundServos.containsKey(device.name())) {
                throw new IllegalStateException("duplicate positional servo declaration: "
                        + device.name());
            }
            Servo servo = hardwareMap.get(Servo.class, device.name());
            servo.setDirection(Servo.Direction.valueOf(device.direction()));
            // Like the archive HoodSubsystem, do not move a positional servo during init;
            // the first explicit command positions it.
            foundServos.put(device.name(), servo);
        }
        // Keep legacy test-only mechanisms usable while proto2 profiles use typed declarations.
        if (mechanism.positionalServos().isEmpty()) {
            for (String name : mechanism.servoNames()) {
                foundServos.put(name, hardwareMap.get(Servo.class, name));
            }
        }
        servos = immutable(foundServos);

        Mechanism.Pinpoint config = mechanism.pinpoint();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // x/y names are measured pod axes: forward pod lateral offset=161, strafe pod
        // forward offset=0. This is intentionally not a spatial-axis swap.
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

    void stopPower() {
        for (DcMotorEx motor : motors.values()) {
            try {
                motor.setPower(0.0);
            } catch (RuntimeException ignored) {
                // Best effort: continue zeroing every other output before surfacing the fault.
            }
        }
        for (CRServo servo : crServos.values()) {
            try {
                servo.setPower(0.0);
            } catch (RuntimeException ignored) {
                // Best effort as above.
            }
        }
    }

    private static void configureDcMotor(DcMotorEx motor, RobotConstants.DcDevice device) {
        motor.setDirection(DcMotorSimple.Direction.valueOf(device.direction()));
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.valueOf(device.zeroPower()));
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private static String wheelHardwareName(String name) {
        return switch (name) {
            case "fl" -> RobotConstants.LEFT_FRONT_MOTOR_NAME;
            case "fr" -> RobotConstants.RIGHT_FRONT_MOTOR_NAME;
            case "bl" -> RobotConstants.LEFT_BACK_MOTOR_NAME;
            case "br" -> RobotConstants.RIGHT_BACK_MOTOR_NAME;
            default -> name;
        };
    }

    private static String encoderNameForWheel(String name) {
        return switch (name) {
            case "fl" -> "leftFront";
            case "fr" -> "rightFront";
            case "bl" -> "leftBack";
            case "br" -> "rightBack";
            default -> name;
        };
    }

    private static void addEncoderAlias(Map<String, DcMotorEx> encoders,
                                        String name, DcMotorEx motor) {
        if (encoders.put(name, motor) != null) {
            throw new IllegalStateException("duplicate encoder declaration: " + name);
        }
    }

    private static void requireFreshName(Map<String, DcMotorEx> motors, String name) {
        if (motors.containsKey(name)) {
            throw new IllegalStateException("duplicate motor declaration: " + name);
        }
    }

    private static <T> Map<String, T> immutable(Map<String, T> values) {
        return Collections.unmodifiableMap(new LinkedHashMap<>(values));
    }
}
