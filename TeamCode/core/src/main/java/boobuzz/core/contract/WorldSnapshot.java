package boobuzz.core.contract;

import com.pedropathing.math.Pose;

/**
 * Dunyanin o anki hali. C1'de sadece poz + oz-durum var.
 *
 * <p>{@code tracks} (izler) C4 ile, {@code rays} (ham ToF isinlari) C5 ile gelir.
 * Simdi yazilmadilar - altindaki katman calismadan ustteki yazilmaz.
 */
public record WorldSnapshot(long t, Pose pose, double yaw, double voltage) {
}
