package boobuzz.core.contract;

import com.pedropathing.math.Pose;

/**
 * Surus niyeti. docs/mimari.md §6.
 *
 * <p>Isaret duzeni Pedro ile ayni: {@code vx} ILERI, {@code vy} SOL, {@code omega} CCW.
 * {@code GamepadController} ham stickleri once {@code (-ly, -lx, -rx)} ile bu duzene cevirir.
 */
public sealed interface Drive {

    /** Robot cercevesi, -1..1 ham guc. C1'de tek desteklenen mod. */
    record Manual(double vx, double vy, double omega) implements Drive {}

    /**
     * Saha cercevesi, in/s. RL bunu kullanir (docs/mimari.md §6).
     * KAPALI CEVRIM olmak zorunda; acik cevrim guc eslemesi kullanilirsa
     * gizliden gizliye "guc" olur ve pil voltajiyla %20+ kayar.
     */
    record Velocity(double vx, double vy, double omega) implements Drive {}

    /** Bir noktaya git. C2.5'te Pedro follower'a baglanir. */
    record GoTo(Pose target, Constraints constraints) implements Drive {}

    /** Onceden tanimli yolu takip et. */
    record FollowPath(String pathId) implements Drive {}

    /** Oldugun yerde dur. */
    record Hold() implements Drive {}

    /** Hareket kisitlari. C1'de kullanilmaz, sozlesme sekli icin burada. */
    record Constraints(double maxPower, double maxVelocity) {
        public static Constraints defaults() {
            return new Constraints(1.0, Double.MAX_VALUE);
        }
    }

    Drive HOLD = new Hold();
}
