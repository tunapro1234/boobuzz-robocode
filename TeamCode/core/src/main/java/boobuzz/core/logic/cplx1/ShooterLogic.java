package boobuzz.core.logic.cplx1;

import boobuzz.core.contract.RequestStatus;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.IShooter;

import com.pedropathing.math.Pose;

import java.util.List;
import java.util.Objects;

/** Owns shot sequencing, while the shooter stub remains a small mechanism. */
public final class ShooterLogic {

    public enum State { IDLE, SPINNING, FEEDING, DONE }

    private final IShooter shooter;
    private final TurretLogic turret;
    private State state = State.IDLE;
    private Pose lastPose;
    private int requestId = -1;
    private int remaining;
    private boolean shot;
    private double rpm;
    private double hood;

    public ShooterLogic(IShooter shooter, TurretLogic turret) {
        this.shooter = Objects.requireNonNull(shooter, "shooter");
        this.turret = Objects.requireNonNull(turret, "turret");
    }

    public void observe(Pose pose) {
        lastPose = pose;
    }

    public RequestStatus requestShot(int id, int count) {
        if (busy()) {
            return RequestStatus.rejected(id, "shooter already has a request");
        }
        if (count <= 0) {
            return RequestStatus.rejected(id, "SHOOT requires a positive count");
        }
        begin(id, count, true, rpmFor(turret.distanceFrom(lastPose)));
        return active(id, 0.0, "spinning up and aiming");
    }

    public RequestStatus requestSpinUp(int id, double targetRpm) {
        if (busy()) {
            return RequestStatus.rejected(id, "shooter already has a request");
        }
        if (!Double.isFinite(targetRpm) || targetRpm <= 0.0) {
            return RequestStatus.rejected(id, "SPIN_UP requires positive rpm");
        }
        begin(id, 0, false, targetRpm);
        return active(id, 0.0, "spinning up");
    }

    public void update(List<RequestStatus> statuses) {
        if (state == State.IDLE) {
            return;
        }
        if (state == State.DONE) {
            state = State.IDLE;
            return;
        }
        shooter.spinUp(rpm);
        if (state == State.SPINNING) {
            if (!shooter.isReady() || (shot && !turret.locked())) {
                statuses.add(active(requestId, shooter.isReady() ? 0.5 : 0.0,
                        shot ? "aiming" : "spinning up"));
                return;
            }
            if (!shot) {
                state = State.DONE;
                statuses.add(RequestStatus.done(requestId));
                return;
            }
            turret.holdForShot(true);
            state = State.FEEDING;
            feed(statuses);
            return;
        }
        feed(statuses);
    }

    public State state() {
        return state;
    }

    public double lastRpm() {
        return rpm;
    }

    public double lastHood() {
        return hood;
    }

    public double rpmFor(double distanceInches) {
        return RobotConstants.SHOOTER_RPM_BASE
                + RobotConstants.SHOOTER_RPM_PER_IN * distanceInches;
    }

    public double hoodFor(double distanceInches) {
        return RobotConstants.SHOOTER_HOOD_BASE
                + RobotConstants.SHOOTER_HOOD_PER_IN * distanceInches;
    }

    public void cancelAll(List<RequestStatus> statuses) {
        if (busy()) {
            statuses.add(RequestStatus.rejected(requestId, "engine switch"));
        }
        shooter.spinDown();
        turret.holdForShot(false);
        state = State.IDLE;
        requestId = -1;
        remaining = 0;
        shot = false;
    }

    private void begin(int id, int count, boolean isShot, double targetRpm) {
        state = State.SPINNING;
        requestId = id;
        remaining = count;
        shot = isShot;
        rpm = targetRpm;
        hood = hoodFor(turret.distanceFrom(lastPose));
        shooter.spinUp(rpm);
    }

    private void feed(List<RequestStatus> statuses) {
        if (shooter.isFeeding()) {
            statuses.add(active(requestId, 0.5, "feeding"));
            return;
        }
        if (remaining > 0) {
            shooter.feed();
            if (shooter.isFeeding()) {
                remaining--;
                statuses.add(active(requestId, 0.5, "feeding"));
            } else {
                statuses.add(active(requestId, 0.0, "feed not accepted"));
            }
            return;
        }
        turret.holdForShot(false);
        state = State.DONE;
        statuses.add(RequestStatus.done(requestId));
    }

    private boolean busy() {
        return state != State.IDLE && state != State.DONE;
    }

    private static RequestStatus active(int id, double progress, String note) {
        return new RequestStatus(id, RequestStatus.State.ACTIVE, progress, note);
    }
}
