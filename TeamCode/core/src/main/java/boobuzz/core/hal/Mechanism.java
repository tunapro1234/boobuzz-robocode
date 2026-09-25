package boobuzz.core.hal;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/** Shared immutable mechanism data for the robot and simulator. */
public record Mechanism(
        List<String> motorNames,
        List<String> servoNames,
        Map<String, Motor> motors,
        Drivetrain drivetrain,
        Pinpoint pinpoint,
        Physics physics,
        List<RobotConstants.DcDevice> dcDevices,
        List<RobotConstants.CrServo> crServos,
        List<RobotConstants.PosServo> positionalServos,
        List<String> encoderNames,
        List<AnalogInput> analogInputs) {

    /** Shared mechanism built once from {@link RobotConstants}. */
    public static final Mechanism DEFAULT = RobotConstants.buildMechanism();

    public Mechanism {
        motorNames = Collections.unmodifiableList(new ArrayList<>(motorNames));
        servoNames = Collections.unmodifiableList(new ArrayList<>(servoNames));
        motors = Collections.unmodifiableMap(new java.util.LinkedHashMap<>(motors));
        dcDevices = Collections.unmodifiableList(new ArrayList<>(dcDevices));
        crServos = Collections.unmodifiableList(new ArrayList<>(crServos));
        positionalServos = Collections.unmodifiableList(new ArrayList<>(positionalServos));
        encoderNames = Collections.unmodifiableList(new ArrayList<>(encoderNames));
        analogInputs = Collections.unmodifiableList(new ArrayList<>(analogInputs));
    }

    /** Legacy six-field constructor for proto1/test-only wheel mechanisms. */
    public Mechanism(List<String> motorNames, List<String> servoNames,
                     Map<String, Motor> motors, Drivetrain drivetrain,
                     Pinpoint pinpoint, Physics physics) {
        this(motorNames, servoNames, motors, drivetrain, pinpoint, physics,
                Collections.emptyList(), Collections.emptyList(), Collections.emptyList(),
                motorNames, Collections.emptyList());
    }

    /** Validates the mechanism schema against names reported by the server. */
    public void requireNames(List<String> actualMotors, List<String> actualServos) {
        List<String> expectedM = sorted(motorNames);
        List<String> gotM = sorted(actualMotors == null
                ? Collections.emptyList() : actualMotors);
        List<String> expectedS = sorted(servoNames);
        List<String> gotS = sorted(actualServos == null
                ? Collections.emptyList() : actualServos);
        if (!expectedM.equals(gotM) || !expectedS.equals(gotS)) {
            throw new MechanismException(
                    "RobotConstants motor/servo name lists do not match.\n"
                            + "  motors expected=" + expectedM + " actual=" + gotM + "\n"
                            + "  servos expected=" + expectedS + " actual=" + gotS);
        }
    }

    public Motor motor(String name) {
        Motor motor = motors.get(name);
        if (motor == null) {
            throw new MechanismException("RobotConstants does not define motor '" + name + "'");
        }
        return motor;
    }

    public RobotConstants.DcDevice dcDevice(String name) {
        for (RobotConstants.DcDevice device : dcDevices) {
            if (device.name().equals(name)) return device;
        }
        throw new MechanismException("RobotConstants does not define DC device '" + name + "'");
    }

    public RobotConstants.CrServo crServo(String name) {
        for (RobotConstants.CrServo servo : crServos) {
            if (servo.name().equals(name)) return servo;
        }
        throw new MechanismException("RobotConstants does not define CR servo '" + name + "'");
    }

    public RobotConstants.PosServo positionalServo(String name) {
        for (RobotConstants.PosServo servo : positionalServos) {
            if (servo.name().equals(name)) return servo;
        }
        throw new MechanismException(
                "RobotConstants does not define positional servo '" + name + "'");
    }

    /** All declared power-device names in ready/step order. */
    public List<String> powerDeviceNames() {
        return motorNames;
    }

    /** Typed DC devices excluding the four wheel Motor records. */
    public List<RobotConstants.DcDevice> dcDevices() {
        return dcDevices;
    }

    public List<RobotConstants.CrServo> crServos() {
        return crServos;
    }

    public List<RobotConstants.PosServo> positionalServos() {
        return positionalServos;
    }

    /** Alias matching the short B01 declaration name. */
    public List<RobotConstants.PosServo> posServos() {
        return positionalServos;
    }

    public List<String> encoderNames() {
        return encoderNames;
    }

    /** Declared analog input names in ready.analogs / state.analog order. */
    public List<String> analogInputNames() {
        List<String> names = new ArrayList<>(analogInputs.size());
        for (AnalogInput input : analogInputs) names.add(input.name());
        return Collections.unmodifiableList(names);
    }

    /** Validates the ordered proto3 ready.analogs list against the declaration. */
    public void requireExactAnalogNames(List<String> actual) {
        List<String> got = actual == null ? Collections.emptyList() : actual;
        if (!analogInputNames().equals(got)) {
            throw new MechanismException(
                    "RobotConstants analog input list does not match exactly.\n"
                            + "  analogs expected=" + analogInputNames() + " actual=" + got);
        }
    }

    public boolean usesProto2() {
        return !dcDevices.isEmpty() || !crServos.isEmpty() || !positionalServos.isEmpty();
    }

    /** Validates ordered ready lists for the versioned seam. */
    public void requireExactNames(List<String> actualMotors, List<String> actualServos) {
        List<String> gotMotors = actualMotors == null
                ? Collections.emptyList() : actualMotors;
        List<String> gotServos = actualServos == null
                ? Collections.emptyList() : actualServos;
        if (!motorNames.equals(gotMotors) || !servoNames.equals(gotServos)) {
            throw new MechanismException(
                    "RobotConstants actuator lists do not match exactly.\n"
                            + "  motors expected=" + motorNames + " actual=" + gotMotors + "\n"
                            + "  servos expected=" + servoNames + " actual=" + gotServos);
        }
    }

    public List<String> wheelMotorNames() {
        List<String> wheels = new ArrayList<>();
        for (String name : motorNames) {
            Motor motor = motors.get(name);
            if (motor != null && "wheel".equals(motor.drives())) {
                wheels.add(name);
            }
        }
        return Collections.unmodifiableList(new ArrayList<>(wheels));
    }

    public Pinpoint pinpoint() {
        if (pinpoint == null) {
            throw new MechanismException("RobotConstants does not define sensors.pinpoint");
        }
        return pinpoint;
    }

    private static List<String> sorted(List<String> values) {
        List<String> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        return sorted;
    }

    /** Motor position: +forward is forward, +left is left. */
    public record Motor(String drives, double forward, double left, double freeRpm) {}

    public record Drivetrain(double wheelDiameter) {}

    /**
     * Analog input and its valid voltage range. A reading outside the range, or a
     * missing reading, is invalid; it is never replaced by 0 V.
     */
    public record AnalogInput(String name, double minVolts, double maxVolts) {
        public boolean isValid(double volts) {
            return Double.isFinite(volts) && volts >= minVolts && volts <= maxVolts;
        }
    }

    public record Pinpoint(double xPodOffsetMm, double yPodOffsetMm,
                           String xPodDirection, String yPodDirection, String podType) {}

    public record Physics(Map<String, Double> efficiency, double strafeEfficiency,
                          double zeroPowerDecelForwardInchesPerSecondSquared,
                          double zeroPowerDecelLateralInchesPerSecondSquared) {}

    public static final class MechanismException extends RuntimeException {
        public MechanismException(String message) {
            super(message);
        }
    }
}
