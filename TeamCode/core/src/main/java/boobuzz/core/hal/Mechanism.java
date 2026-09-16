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
        Physics physics) {

    public Mechanism {
        motorNames = List.copyOf(motorNames);
        servoNames = List.copyOf(servoNames);
        motors = Map.copyOf(motors);
    }

    /** Validates the mechanism schema against names reported by the server. */
    public void requireNames(List<String> actualMotors, List<String> actualServos) {
        List<String> expectedM = sorted(motorNames);
        List<String> gotM = sorted(actualMotors == null ? List.of() : actualMotors);
        List<String> expectedS = sorted(servoNames);
        List<String> gotS = sorted(actualServos == null ? List.of() : actualServos);
        if (!expectedM.equals(gotM) || !expectedS.equals(gotS)) {
            throw new MechanismException(
                    "mechanism.yaml motor/servo name lists do not match.\n"
                            + "  motors expected=" + expectedM + " actual=" + gotM + "\n"
                            + "  servos expected=" + expectedS + " actual=" + gotS);
        }
    }

    public Motor motor(String name) {
        Motor motor = motors.get(name);
        if (motor == null) {
            throw new MechanismException("mechanism.yaml does not define motor '" + name + "'");
        }
        return motor;
    }

    public List<String> wheelMotorNames() {
        List<String> wheels = new ArrayList<>();
        for (String name : motorNames) {
            if ("wheel".equals(motors.get(name).drives())) {
                wheels.add(name);
            }
        }
        return List.copyOf(wheels);
    }

    public Pinpoint pinpoint() {
        if (pinpoint == null) {
            throw new MechanismException("mechanism.yaml does not define sensors.pinpoint");
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
