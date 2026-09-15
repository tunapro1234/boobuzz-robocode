package boobuzz.core.hal;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

/** Robot ve sim tarafinin ortak, degismez mekanizma verisi. */
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

    /** Sunucunun bildirdigi donanim adlariyla mekanizma semasini dogrular. */
    public void requireNames(List<String> actualMotors, List<String> actualServos) {
        List<String> expectedM = sorted(motorNames);
        List<String> gotM = sorted(actualMotors == null ? List.of() : actualMotors);
        List<String> expectedS = sorted(servoNames);
        List<String> gotS = sorted(actualServos == null ? List.of() : actualServos);
        if (!expectedM.equals(gotM) || !expectedS.equals(gotS)) {
            throw new MechanismException(
                    "mechanism.yaml ile sunucu ad listesi uyusmuyor.\n"
                            + "  motor  beklenen=" + expectedM + " gelen=" + gotM + "\n"
                            + "  servo  beklenen=" + expectedS + " gelen=" + gotS);
        }
    }

    public Motor motor(String name) {
        Motor motor = motors.get(name);
        if (motor == null) {
            throw new MechanismException("mechanism.yaml'da '" + name + "' motoru yok");
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
            throw new MechanismException("mechanism.yaml'da sensors.pinpoint yok");
        }
        return pinpoint;
    }

    private static List<String> sorted(List<String> values) {
        List<String> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        return sorted;
    }

    /** Motor konumu: +forward ileri, +left sol. */
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
