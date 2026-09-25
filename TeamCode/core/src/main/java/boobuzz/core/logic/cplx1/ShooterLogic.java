package boobuzz.core.logic.cplx1;

import boobuzz.core.contract.RequestStatus;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.logic.MechanismProfile;
import boobuzz.core.logic.ShooterCalibration;
import boobuzz.core.logic.shot.ShotPreset;
import boobuzz.core.subsystem.IShooter;

import com.pedropathing.math.Pose;

import java.util.List;
import java.util.Objects;

/**
 * Shared shot coordinator: owns flywheel/hood/feeder sequencing and the intake while a
 * shot is active. Every pulse requires measured shooter readiness, estimated hood
 * settling, a valid settled turret aim and a stationary chassis; targets are latched per
 * pulse and rechecked before the next one.
 */
public final class ShooterLogic {

    public enum State { IDLE, PREPARE, FEED, RECOVER, COMPLETE, FAULT }

    private final IShooter shooter;
    private final TurretLogic turret;
    private final MechanismProfile profile;
    private final PoseMotionEstimator motion = new PoseMotionEstimator();
    private ShotPreset preset = ShotPreset.DEFAULT;
    private boolean presetExplicit;
    private boolean presetPending;
    private State state = State.IDLE;
    private long nowMs;
    private Pose lastPose;
    private int requestId = -1;
    private int count;
    private int remaining;
    private boolean shot;
    private boolean stopping;
    private double explicitRpm = Double.NaN;
    private double rpm;
    private double hood;
    private boolean hoodCommanded;
    private boolean turretOwned;
    private long prepareSinceMs = -1;

    public ShooterLogic(IShooter shooter, TurretLogic turret) {
        this(shooter, turret, MechanismProfile.STUB);
    }

    public ShooterLogic(IShooter shooter, TurretLogic turret, MechanismProfile profile) {
        this.shooter = Objects.requireNonNull(shooter, "shooter");
        this.turret = Objects.requireNonNull(turret, "turret");
        this.profile = Objects.requireNonNull(profile, "profile");
    }

    /** One Pinpoint sample per tick; drives the stationary gate. */
    public void observe(long tMs, Pose pose) {
        nowMs = tMs;
        lastPose = pose;
        motion.observe(tMs, pose);
    }

    /** Explicit localization reset: the motion window restarts. */
    public void localizationReset() {
        motion.reset();
    }

    /** Count-only SHOOT: RPM comes from the profile (preset, or legacy stub calibration). */
    public RequestStatus requestShot(int id, int count) {
        return beginShot(id, count, Double.NaN);
    }

    /** SHOOT with an explicit RPM. */
    public RequestStatus requestShot(int id, int count, double targetRpm) {
        if (!Double.isFinite(targetRpm) || targetRpm <= 0.0) {
            return RequestStatus.rejected(id, "SHOOT requires a positive finite rpm");
        }
        return beginShot(id, count, targetRpm);
    }

    private RequestStatus beginShot(int id, int count, double targetRpm) {
        if (busy()) {
            return RequestStatus.rejected(id, "shooter already has a request");
        }
        if (count <= 0 || count > RobotConstants.SHOT_MAX_COUNT) {
            return RequestStatus.rejected(id,
                    "SHOOT count must be 1.." + RobotConstants.SHOT_MAX_COUNT);
        }
        begin(id, count, true, targetRpm);
        return active(id, 0.0, "preparing");
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

    /** SET_SHOT_PRESET: validated, never clamped; an in-flight pulse keeps its latched values. */
    public RequestStatus setPreset(int id, double presetRpm, double hoodDeg, double turretRad) {
        String problem = ShotPreset.validate(presetRpm, hoodDeg, turretRad);
        if (problem != null) {
            return RequestStatus.rejected(id, problem);
        }
        preset = new ShotPreset(presetRpm, hoodDeg, turretRad);
        presetExplicit = true;
        if (shot && busy()) {
            if (state == State.FEED) {
                presetPending = true;
            } else {
                latchTargets();
            }
        }
        return RequestStatus.done(id);
    }

    /** STOP_SHOOTING: finish the current pulse, start no new ones. Always accepted. */
    public RequestStatus stopShooting(int id) {
        if (shot && busy()) {
            stopping = true;
        }
        return RequestStatus.done(id);
    }

    public void update(List<RequestStatus> statuses) {
        switch (state) {
            case IDLE -> { }
            case COMPLETE, FAULT -> state = State.IDLE;
            case FEED -> updateFeed(statuses);
            case PREPARE, RECOVER -> updatePrepare(statuses);
        }
    }

    public State state() {
        return state;
    }

    public ShotPreset preset() {
        return preset;
    }

    /** True while a SHOOT request owns the intake output. */
    public boolean ownsIntake() {
        return shot && busy();
    }

    /** True while an active shot latched its own turret target (preset relative angle). */
    public boolean ownsTurretTarget() {
        return shot && busy() && turretOwned;
    }

    public double lastRpm() {
        return rpm;
    }

    public double lastHood() {
        return hood;
    }

    public PoseMotionEstimator motion() {
        return motion;
    }

    /** Legacy count-only shot speed (STUB profile only). */
    public static double calibratedRpm(Pose pose) {
        return ShooterCalibration.calibratedRpm(pose);
    }

    public void cancelAll(List<RequestStatus> statuses) {
        if (busy()) {
            statuses.add(RequestStatus.rejected(requestId, "engine switch"));
        }
        stop();
    }

    /** Cancels whatever request is active (e.g. an incompatible turret retarget). */
    public void cancelActive(String note, List<RequestStatus> statuses) {
        if (busy()) {
            statuses.add(RequestStatus.rejected(requestId, note));
        }
        stop();
    }

    /** Cancels one request owned by this shooter, if it is currently active. */
    public boolean cancel(int id, List<RequestStatus> statuses) {
        if (!busy() || requestId != id) {
            return false;
        }
        stop();
        statuses.add(RequestStatus.rejected(id, "cancelled"));
        return true;
    }

    private void stop() {
        shooter.spinDown();
        turret.holdForShot(false);
        state = State.IDLE;
        requestId = -1;
        remaining = 0;
        shot = false;
        stopping = false;
        presetPending = false;
        turretOwned = false;
    }

    private void begin(int id, int shots, boolean isShot, double targetRpm) {
        state = State.PREPARE;
        requestId = id;
        count = shots;
        remaining = shots;
        shot = isShot;
        stopping = false;
        presetPending = false;
        explicitRpm = targetRpm;
        prepareSinceMs = -1;
        latchTargets();
        if (shot && !turretOwned) {
            turret.enable();
        }
    }

    /** Targets for the next pulse; never called while a pulse is in flight. */
    private void latchTargets() {
        boolean usePreset = profile == MechanismProfile.REAL || presetExplicit;
        if (!shot) {
            rpm = explicitRpm;
            hoodCommanded = false;
            turretOwned = false;
        } else if (usePreset) {
            rpm = Double.isNaN(explicitRpm) ? preset.rpm() : explicitRpm;
            hood = preset.hoodDeg();
            hoodCommanded = true;
            turretOwned = true;
            turret.setRelativeTarget(preset.turretRad());
        } else {
            rpm = Double.isNaN(explicitRpm) ? calibratedRpm(lastPose) : explicitRpm;
            hood = RobotConstants.SHOOTER_HOOD_BASE
                    + RobotConstants.SHOOTER_HOOD_PER_IN * turret.distanceFrom(lastPose);
            hoodCommanded = false;
            turretOwned = false;
        }
        commandTargets();
    }

    private void commandTargets() {
        shooter.spinUp(rpm);
        if (hoodCommanded) {
            shooter.setHoodAngleDeg(hood);
        }
    }

    private void updatePrepare(List<RequestStatus> statuses) {
        commandTargets();
        if (!shot) {
            if (shooter.isReady()) {
                finish(statuses, RequestStatus.done(requestId));
            } else {
                statuses.add(active(requestId, 0.0, "spinning up"));
            }
            return;
        }
        if (remaining == 0 || stopping) {
            finish(statuses, remaining == 0 ? RequestStatus.done(requestId)
                    : new RequestStatus(requestId, RequestStatus.State.DONE, progress(), "stopped"));
            return;
        }
        String blocked = blockReason();
        if (blocked != null) {
            if (turret.startupDone() && prepareSinceMs < 0) {
                prepareSinceMs = nowMs;
            }
            if (prepareSinceMs >= 0 && nowMs - prepareSinceMs > RobotConstants.SHOT_PREPARE_TIMEOUT_MS) {
                RequestStatus failed = new RequestStatus(requestId, RequestStatus.State.FAILED,
                        progress(), "prepare timeout: " + blocked);
                shooter.spinDown();
                turret.holdForShot(false);
                finishWith(State.FAULT, statuses, failed);
                return;
            }
            statuses.add(active(requestId, progress(), blocked));
            return;
        }
        turret.holdForShot(true);
        shooter.feed();
        if (shooter.isFeeding()) {
            remaining--;
            state = State.FEED;
            statuses.add(active(requestId, progress(), "feeding"));
        } else {
            turret.holdForShot(false);
            statuses.add(active(requestId, progress(), "feed not accepted"));
        }
    }

    private void updateFeed(List<RequestStatus> statuses) {
        commandTargets();
        if (shooter.isFeeding()) {
            statuses.add(active(requestId, progress(), "feeding"));
            return;
        }
        turret.holdForShot(false);
        state = State.RECOVER;
        prepareSinceMs = -1;
        if (presetPending) {
            presetPending = false;
            latchTargets();
        }
        updatePrepare(statuses);
    }

    /** Null when every feed gate passes, otherwise the first blocking reason. */
    private String blockReason() {
        if (!shooter.isReady()) {
            return "spinning up";
        }
        if (hoodCommanded && !shooter.hoodSettled()) {
            return "hood settling";
        }
        if (!turret.locked()) {
            return "aiming";
        }
        if (!motion.stationary(nowMs)) {
            return "chassis moving";
        }
        return null;
    }

    private void finish(List<RequestStatus> statuses, RequestStatus terminal) {
        turret.holdForShot(false);
        finishWith(State.COMPLETE, statuses, terminal);
    }

    private void finishWith(State next, List<RequestStatus> statuses, RequestStatus terminal) {
        statuses.add(terminal);
        state = next;
        requestId = -1;
        remaining = 0;
        shot = false;
        stopping = false;
        presetPending = false;
        turretOwned = false;
    }

    private double progress() {
        return count <= 0 ? 0.0 : (double) (count - remaining) / count;
    }

    private boolean busy() {
        return state == State.PREPARE || state == State.FEED || state == State.RECOVER;
    }

    private static RequestStatus active(int id, double progress, String note) {
        return new RequestStatus(id, RequestStatus.State.ACTIVE, progress, note);
    }
}
