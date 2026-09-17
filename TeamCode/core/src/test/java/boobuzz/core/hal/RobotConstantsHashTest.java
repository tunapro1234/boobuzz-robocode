package boobuzz.core.hal;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

public class RobotConstantsHashTest {

    @Test
    public void changingOnlyRobotMassChangesConstantsHash() {
        assertEquals(RobotConstants.constantsHash(),
                RobotConstants.constantsHashForMass(RobotConstants.ROBOT_MASS_KG));
        assertNotEquals(RobotConstants.constantsHashForMass(12.0),
                RobotConstants.constantsHashForMass(18.0));
    }
}
