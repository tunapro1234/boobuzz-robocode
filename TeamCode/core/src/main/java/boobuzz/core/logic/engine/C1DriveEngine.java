package boobuzz.core.logic.engine;

import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.hal.RobotAction;
import boobuzz.core.hal.RobotState;
import boobuzz.core.mechanism.Mechanism;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * C1 - en basit Engine. Localizer (salt odometri) + manuel surus. Baska hicbir sey.
 *
 * <p>Bu fazin sonunda surulebilir bir robot vardir. Subsystem, vision, world model
 * C2/C3/C4 ile gelir; hicbiri altindaki katman robotta calismadan yazilmaz.
 *
 * <p>Motor adlari {@code mechanism.yaml}'dan gelir; dort tekerlekli mecanum icin
 * {@code drives: wheel} olan motorlar {@code pos} alanlarina gore on/arka sol/sag
 * olarak yerlestirilir. Boylece adlar degisse de kod degismez.
 */
public final class C1DriveEngine implements RobotEngine {

    private final Mechanism mechanism;
    private final String frontLeft;
    private final String frontRight;
    private final String backLeft;
    private final String backRight;

    private List<RequestStatus> pendingStatuses = List.of();

    public C1DriveEngine(Mechanism mechanism) {
        this.mechanism = mechanism;
        List<String> wheels = mechanism.wheelMotorNames();
        if (wheels.size() != 4) {
            throw new Mechanism.MechanismException(
                    "C1DriveEngine dort adet 'drives: wheel' motoru bekler, " + wheels.size()
                            + " buldu: " + wheels);
        }
        // pos = [x ileri, y sol] (docs/protokol.md cerceve kurali).
        // Koseler adlardan degil KONUMDAN turetilir; ad degisse de kod degismez.
        String fl = null, fr = null, bl = null, br = null;
        for (String name : wheels) {
            Mechanism.Motor m = mechanism.motor(name);
            boolean front = m.forward() >= 0;
            boolean left = m.left() >= 0;
            if (front && left) {
                fl = pick(fl, name, "on-sol");
            } else if (front) {
                fr = pick(fr, name, "on-sag");
            } else if (left) {
                bl = pick(bl, name, "arka-sol");
            } else {
                br = pick(br, name, "arka-sag");
            }
        }
        if (fl == null || fr == null || bl == null || br == null) {
            throw new Mechanism.MechanismException(
                    "Tekerlek konumlari dort ayri koseye dusmuyor. pos alanlarini kontrol et: "
                            + wheels);
        }
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backLeft = bl;
        this.backRight = br;
    }

    private static String pick(String existing, String name, String corner) {
        if (existing != null) {
            throw new Mechanism.MechanismException(
                    "Iki motor ayni koseye (" + corner + ") dusuyor: " + existing + " ve " + name);
        }
        return name;
    }

    @Override
    public String name() {
        return "C1Drive";
    }

    @Override
    public Feedback sense(long now, RobotState state) {
        WorldSnapshot world = new WorldSnapshot(
                state.t(), state.pinpoint(), state.yaw(), state.voltage());
        Feedback feedback = new Feedback(world, pendingStatuses, now);
        pendingStatuses = List.of();
        return feedback;
    }

    @Override
    public RobotAction act(Intent intent) {
        // C1'de subsystem yok: her istek reddedilir. Sessizce yutulmaz, rapor edilir.
        if (!intent.newRequests().isEmpty()) {
            List<RequestStatus> statuses = new ArrayList<>(intent.newRequests().size());
            for (Request r : intent.newRequests()) {
                statuses.add(RequestStatus.rejected(r.id(), "C1'de subsystem yok"));
            }
            pendingStatuses = List.copyOf(statuses);
        }

        double vx = 0, vy = 0, omega = 0;
        if (intent.drive() instanceof Drive.Manual m) {
            vx = m.vx();
            vy = m.vy();
            omega = m.omega();
        }
        // Velocity / GoTo / FollowPath C2.5'te Pedro follower ile gelir; C1'de dururuz.

        return new RobotAction(mecanum(vx, vy, omega), Map.of());
    }

    /**
     * Mecanum ters kinematigi. Isaret duzeni: vx ILERI, vy SOL, omega CCW.
     *
     * <p>Toplam 1'i asarsa hepsi ayni oranda kucultulur - yon korunur, sadece
     * hiz duser. Tek tek kirpmak robotun komut edilen yonden sapmasina yol acar.
     */
    private Map<String, Double> mecanum(double vx, double vy, double omega) {
        double fl = vx - vy - omega;
        double fr = vx + vy + omega;
        double bl = vx + vy - omega;
        double br = vx - vy + omega;

        double peak = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (peak > 1.0) {
            fl /= peak;
            fr /= peak;
            bl /= peak;
            br /= peak;
        }

        Map<String, Double> out = new LinkedHashMap<>(4);
        out.put(frontLeft, fl);
        out.put(frontRight, fr);
        out.put(backLeft, bl);
        out.put(backRight, br);
        return out;
    }

    // Testler ve telemetri icin.
    public String frontLeftMotor() { return frontLeft; }

    public String frontRightMotor() { return frontRight; }

    public String backLeftMotor() { return backLeft; }

    public String backRightMotor() { return backRight; }

    public Mechanism mechanism() { return mechanism; }
}
