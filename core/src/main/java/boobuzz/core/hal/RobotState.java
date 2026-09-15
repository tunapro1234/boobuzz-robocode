package boobuzz.core.hal;

import com.pedropathing.math.Pose;

import java.util.Map;

/**
 * YUKARI gelen ham sensor okumasi. docs/protokol.md ile birebir.
 *
 * <p>{@code t} sim/robot saatidir ve {@link Hal#now()} ile ayni kaynaktir
 * (anayasa kural 2: zaman HAL'den).
 *
 * <p>Simulatordeki {@code truth} alani BURAYA GIRMEZ - :core gercegi hic gormez,
 * yoksa simde calisip robotta calismayan kod yaziliriz.
 *
 * @param t       milisaniye, HAL saati
 * @param enc     motor adi -> enkoder tick (tam sayi)
 * @param vel     motor adi -> tick/saniye
 * @param yaw     IMU yaw, RADYAN
 * @param pinpoint odometri pozu (inc, radyan)
 * @param voltage bus voltaji
 */
public record RobotState(long t,
                         Map<String, Integer> enc,
                         Map<String, Double> vel,
                         double yaw,
                         Pose pinpoint,
                         double voltage) {

    public RobotState {
        enc = Map.copyOf(enc);
        vel = Map.copyOf(vel);
    }

    public int enc(String name) {
        return enc.getOrDefault(name, 0);
    }

    public double vel(String name) {
        return vel.getOrDefault(name, 0.0);
    }
}
