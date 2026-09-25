package boobuzz.core.logic.shot;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

public class ShotPresetTest {

    @Test
    public void defaultIsArchiveRecoveryPreset() {
        assertEquals(4000.0, ShotPreset.DEFAULT.rpm(), 0.0);
        assertEquals(45.0, ShotPreset.DEFAULT.hoodDeg(), 0.0);
        assertEquals(0.0, ShotPreset.DEFAULT.turretRad(), 0.0);
    }

    @Test
    public void validationRejectsInsteadOfClamping() {
        assertNull(ShotPreset.validate(4000, 25, Math.toRadians(-90)));
        assertNull(ShotPreset.validate(4000, 50, Math.toRadians(90)));
        assertNotNull(ShotPreset.validate(0, 45, 0));
        assertNotNull(ShotPreset.validate(Double.NaN, 45, 0));
        assertNotNull(ShotPreset.validate(4000, 24.9, 0));
        assertNotNull(ShotPreset.validate(4000, 50.1, 0));
        assertNotNull(ShotPreset.validate(4000, 45, Math.toRadians(91)));
        assertNotNull(ShotPreset.validate(4000, 45, Double.POSITIVE_INFINITY));
    }

    @Test(expected = IllegalArgumentException.class)
    public void constructorRejectsInvalid() {
        new ShotPreset(4000, 60, 0);
    }
}
