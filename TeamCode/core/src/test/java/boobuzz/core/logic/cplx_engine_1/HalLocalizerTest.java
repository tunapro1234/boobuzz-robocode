package boobuzz.core.logic.cplx_engine_1;

import boobuzz.core.hal.RobotState;
import boobuzz.core.logic.cplx_engine_1.HalLocalizer;

import com.pedropathing.math.Pose;

import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertEquals;

public class HalLocalizerTest {

    private static final double EPS = 1e-9;

    @Test
    public void pinpointPozunuOlduguGibiGecirir() {
        HalLocalizer localizer = new HalLocalizer();
        localizer.feed(state(0, 12.5, 31.0, 0.75));

        assertEquals(12.5, localizer.pose().x(), EPS);
        assertEquals(31.0, localizer.pose().y(), EPS);
        assertEquals(0.75, localizer.pose().heading(), EPS);
    }

    @Test
    public void ikiOrnektenSahaHiziniHesaplar() {
        HalLocalizer localizer = new HalLocalizer();
        localizer.feed(state(100, 10, 20, 0));
        localizer.feed(state(120, 11, 20, 0));

        assertEquals(50.0, localizer.velocity().vx, EPS);
        assertEquals(0.0, localizer.velocity().vy, EPS);
        assertEquals(0.0, localizer.velocity().omega, EPS);
        assertEquals(50.0, localizer.twist().vx, EPS);
    }

    @Test
    public void headingFarkiniPiSinirindaSarar() {
        HalLocalizer localizer = new HalLocalizer();
        localizer.feed(state(0, 0, 0, 3.1));
        localizer.feed(state(20, 0, 0, -3.1));

        double smallDifference = 2.0 * Math.PI - 6.2;
        assertEquals(smallDifference / 0.020, localizer.velocity().omega, EPS);
    }

    @Test
    public void setPosePinpointUstundeKaliciOffsetTutar() {
        HalLocalizer localizer = new HalLocalizer();
        localizer.feed(state(0, 10, 20, 0.2));
        localizer.setPose(new Pose(100, 50, 1.0));

        assertEquals(100.0, localizer.pose().x(), EPS);
        assertEquals(50.0, localizer.pose().y(), EPS);
        assertEquals(1.0, localizer.pose().heading(), EPS);

        localizer.feed(state(20, 11, 22, 0.3));
        assertEquals(101.0, localizer.pose().x(), EPS);
        assertEquals(52.0, localizer.pose().y(), EPS);
        assertEquals(1.1, localizer.pose().heading(), EPS);
    }

    @Test
    public void headingOffsetiSahaHiziniDondurupRobotTwistiniKorur() {
        HalLocalizer localizer = new HalLocalizer();
        localizer.feed(state(0, 0, 0, 0));
        localizer.setPose(new Pose(0, 0, Math.PI / 2.0));
        localizer.feed(state(20, 1, 0, 0));

        assertEquals(0.0, localizer.velocity().vx, EPS);
        assertEquals(50.0, localizer.velocity().vy, EPS);
        assertEquals(50.0, localizer.twist().vx, EPS);
        assertEquals(0.0, localizer.twist().vy, EPS);
    }

    @Test
    public void headingPiBoluIkideSahaHiziniRobotCercevesineDondurur() {
        HalLocalizer localizer = new HalLocalizer();
        localizer.feed(state(0, 0, 0, Math.PI / 2.0));
        localizer.feed(state(20, 1, 2, Math.PI / 2.0));

        assertEquals(50.0, localizer.velocity().vx, EPS);
        assertEquals(100.0, localizer.velocity().vy, EPS);
        assertEquals(100.0, localizer.twist().vx, EPS);
        assertEquals(-50.0, localizer.twist().vy, EPS);
    }

    private static RobotState state(long t, double x, double y, double heading) {
        return new RobotState(t, Map.of(), Map.of(), heading,
                new Pose(x, y, heading), 12.6);
    }
}
