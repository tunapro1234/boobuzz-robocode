package boobuzz.core.hal;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * ASAGI giden komut. Motor/servo seviyesi - subsystem seviyesi DEGIL.
 *
 * <p>Seam burada cunku motor arayuzu fizigi kodlar (degismez); subsystem arayuzu
 * tasarim varsayimini kodlar (her hafta degisir). Bkz. docs/mimari.md §3.
 *
 * <p>Anahtarlar {@code mechanism.yaml}'daki adlardir. Motor eklemek :core'u degistirmez.
 * Degerler -1..1 guctur. Eksik anahtar = 0.
 */
public record RobotAction(Map<String, Double> motors, Map<String, Double> servos) {

    public RobotAction {
        motors = Map.copyOf(motors);
        servos = Map.copyOf(servos);
    }

    public static RobotAction zero() {
        return new RobotAction(Collections.emptyMap(), Collections.emptyMap());
    }

    public static RobotAction ofMotors(Map<String, Double> motors) {
        return new RobotAction(motors, Collections.emptyMap());
    }

    public double motor(String name) {
        return motors.getOrDefault(name, 0.0);
    }

    public double servo(String name) {
        return servos.getOrDefault(name, 0.0);
    }

    /** Elle kurmak icin kucuk yardimci; sirayi korur (telemetri okunabilirligi). */
    public static final class Builder {
        private final Map<String, Double> motors = new LinkedHashMap<>();
        private final Map<String, Double> servos = new LinkedHashMap<>();

        public Builder motor(String name, double power) {
            motors.put(name, power);
            return this;
        }

        public Builder servo(String name, double position) {
            servos.put(name, position);
            return this;
        }

        public RobotAction build() {
            return new RobotAction(motors, servos);
        }
    }
}
