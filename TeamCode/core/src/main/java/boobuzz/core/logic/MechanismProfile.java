package boobuzz.core.logic;

/**
 * Which mechanism set the engines drive. STUB keeps the historical count-only shot
 * placeholders (ShooterCalibration RPM, no hood command); REAL uses the shared ShotPreset.
 */
public enum MechanismProfile {
    STUB,
    REAL
}
