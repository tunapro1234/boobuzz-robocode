package boobuzz.core.contract;

import org.junit.Test;

public class PathRequestValidationTest {

    @Test(expected = IllegalArgumentException.class)
    public void zeroPowerIsRejected() {
        new PathRequest.Constraints(0.0, 10.0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void powerAboveOneIsRejected() {
        new PathRequest.Constraints(1.1, 10.0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void nonFiniteVelocityIsRejected() {
        new PathRequest.Constraints(1.0, Double.NaN);
    }

    @Test(expected = IllegalArgumentException.class)
    public void negativeBrakingIsRejected() {
        new PathRequest.Braking(-0.1, 0.5);
    }

    @Test(expected = IllegalArgumentException.class)
    public void brakingAboveOneIsRejected() {
        new PathRequest.Braking(0.5, 1.1);
    }

    @Test(expected = IllegalArgumentException.class)
    public void nonFiniteBrakingIsRejected() {
        new PathRequest.Braking(Double.POSITIVE_INFINITY, 0.5);
    }
}
