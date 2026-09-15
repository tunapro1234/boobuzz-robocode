package boobuzz.sim;

import org.junit.Test;

import static org.junit.Assert.assertThrows;

public class SimMainTest {

    @Test
    public void pathVeDriveBirlikteReddedilir() {
        assertThrows(IllegalArgumentException.class, () -> SimMain.main(new String[] {
                "--path", "test-line",
                "--drive", "1,0,0"}));
    }
}
