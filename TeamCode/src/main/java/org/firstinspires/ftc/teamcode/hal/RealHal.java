package org.firstinspires.ftc.teamcode.hal;

import boobuzz.core.contract.GamepadState;
import boobuzz.core.hal.Hal;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.hal.Mechanism;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

/** Real L1 adapter from FTC devices to the pure-Java {@link Hal} contract. */
public final class RealHal implements Hal {

    private final Mechanism mechanism;
    private final Gamepad gamepad;
    private final Hardware hardware;
    private final long startNanos = System.nanoTime();

    public RealHal(HardwareMap hardwareMap, Gamepad gamepad,
                   Mechanism mechanism, Pose startPose) {
        this.mechanism = Objects.requireNonNull(mechanism, "mechanism");
        this.gamepad = Objects.requireNonNull(gamepad, "gamepad");
        hardware = new Hardware(hardwareMap, mechanism, startPose);
    }

    @Override
    public long now() {
        return (System.nanoTime() - startNanos) / 1_000_000L;
    }

    @Override
    public RobotState read() {
        hardware.pinpoint.update();

        Map<String, Integer> encoders = new LinkedHashMap<>();
        Map<String, Double> velocities = new LinkedHashMap<>();
        for (Map.Entry<String, DcMotorEx> entry : hardware.motors.entrySet()) {
            encoders.put(entry.getKey(), entry.getValue().getCurrentPosition());
            velocities.put(entry.getKey(), entry.getValue().getVelocity());
        }

        double heading = hardware.pinpoint.getHeading(AngleUnit.RADIANS);
        Pose pinpoint = new Pose(
                hardware.pinpoint.getPosX(DistanceUnit.INCH),
                hardware.pinpoint.getPosY(DistanceUnit.INCH),
                heading);
        return new RobotState(now(), encoders, velocities, heading, pinpoint, voltage());
    }

    @Override
    public void write(RobotAction action) {
        for (String name : mechanism.motorNames()) {
            hardware.motors.get(name).setPower(
                    RobotAction.clamp(action.motor(name), -1.0, 1.0));
        }
        for (String name : mechanism.servoNames()) {
            Servo servo = hardware.servos.get(name);
            servo.setPosition(RobotAction.clamp(action.servo(name), 0.0, 1.0));
        }
    }

    @Override
    public GamepadState get() {
        return new GamepadState(
                gamepad.left_stick_x, gamepad.left_stick_y,
                gamepad.right_stick_x, gamepad.right_stick_y,
                gamepad.a, gamepad.b, gamepad.x, gamepad.y,
                gamepad.left_bumper, gamepad.right_bumper,
                gamepad.left_trigger, gamepad.right_trigger,
                dpad(gamepad));
    }

    private double voltage() {
        double voltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardware.voltageSensors) {
            double reading = sensor.getVoltage();
            if (reading > 0.0) {
                voltage = Math.min(voltage, reading);
            }
        }
        if (!Double.isFinite(voltage)) {
            throw new IllegalStateException("could not read a valid robot voltage");
        }
        return voltage;
    }

    private static GamepadState.Dpad dpad(Gamepad gamepad) {
        if (gamepad.dpad_up) return GamepadState.Dpad.UP;
        if (gamepad.dpad_down) return GamepadState.Dpad.DOWN;
        if (gamepad.dpad_left) return GamepadState.Dpad.LEFT;
        if (gamepad.dpad_right) return GamepadState.Dpad.RIGHT;
        return GamepadState.Dpad.NONE;
    }

}
