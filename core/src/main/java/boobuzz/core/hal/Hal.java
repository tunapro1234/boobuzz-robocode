package boobuzz.core.hal;

/**
 * L1 - donanim soyutlamasi. Sim ile gercek arasindaki TEK fark budur.
 *
 * <p>Iki implementasyon: {@code RealHal} (TeamCode/Android) ve {@code SimHal}
 * (:sim, soketle Python fizik sunucusuna). Bu arayuzun ustunde
 * {@code if (isSim)} YOKTUR (anayasa kural 3).
 */
public interface Hal extends GamepadSource {

    /** HAL saati, milisaniye. Sim hizlandirilabilsin diye duvar saati degil. */
    long now();

    /** Sensorleri oku. */
    RobotState read();

    /** Motor/servo komutlarini uygula. */
    void write(RobotAction action);
}
