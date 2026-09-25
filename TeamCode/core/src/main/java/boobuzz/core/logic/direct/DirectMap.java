package boobuzz.core.logic.direct;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestType;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.logic.MechanismProfile;
import boobuzz.core.logic.ShooterCalibration;
import boobuzz.core.logic.shot.ShotPreset;
import boobuzz.core.subsystem.ITurret;
import boobuzz.core.subsystem.Subsystems;

import com.pedropathing.math.Pose;

import java.util.List;

/**
 * Direct request-to-subsystem map. To add a request type: (1) RequestType enum,
 * (2) one case here, (3) done.
 */
public final class DirectMap {

    private final Subsystems subsystems;
    private final MechanismProfile profile;
    private ShotPreset preset = ShotPreset.DEFAULT;
    private boolean presetExplicit;
    private boolean feedHeld;
    private long nowMs;

    public DirectMap(Subsystems subsystems) {
        this(subsystems, MechanismProfile.STUB);
    }

    public DirectMap(Subsystems subsystems, MechanismProfile profile) {
        this.subsystems = subsystems;
        this.profile = profile;
    }

    /** Loop time for the turret startup bound of a gated shot. */
    public void observe(long nowMs) {
        this.nowMs = nowMs;
    }

    /** MECHANISM_RECOVERY owns the feeder: an active shot starts no new pulse. */
    public void holdFeed(boolean hold) {
        feedHeld = hold;
    }

    /**
     * STOP_SHOOTING: the active shot finishes its current pulse, starts no new ones and
     * spins the flywheel down when it ends (archive: releasing RT disables the shooter).
     * Returns true when the job ended here (a SPIN_UP), false when it keeps running.
     */
    public boolean stopShooting(Job job, List<RequestStatus> statuses) {
        if (job instanceof ShootJob shoot) {
            shoot.stopped = shoot.remaining > 0;
            shoot.remaining = 0;
            shoot.stopping = true;
            return false;
        }
        if (job instanceof SpinJob spin) {
            subsystems.shooter().spinDown();
            statuses.add(new RequestStatus(spin.id(), RequestStatus.State.DONE, 0.0, "stopped"));
            return true;
        }
        subsystems.shooter().spinDown();
        return false;
    }

    /** Starts one request and returns a job when it remains active. */
    public Job start(Request request, boolean driveBusy, boolean shooterBusy,
                     List<RequestStatus> statuses) {
        RequestType type = request.type();
        if (isDrive(type) && driveBusy) {
            statuses.add(RequestStatus.rejected(request.id(), "drive already has a request"));
            return null;
        }
        if (isShooter(type) && shooterBusy) {
            statuses.add(RequestStatus.rejected(request.id(), "shooter already has a request"));
            return null;
        }
        return switch (type) {
            case SHOOT -> startShoot(request, statuses);
            case SPIN_UP -> startSpin(request, statuses);
            case GOTO -> startGoto(request, statuses);
            case PATH -> startPath(request, statuses);
            case TURN_TO -> startTurn(request, statuses);
            case INTAKE, INTAKE_ON, INTAKE_OFF -> startIntake(request, statuses);
            case TURRET_AIM -> startTurretAim(request, statuses);
            case RESET_POSE -> startResetPose(request, statuses);
            case SET_SHOT_PRESET -> setPreset(request, statuses);
            default -> {
                statuses.add(RequestStatus.rejected(request.id(), "unsupported request"));
                yield null;
            }
        };
    }

    /** Advances one active job and appends its current status. */
    public boolean advance(Job job, List<RequestStatus> statuses) {
        if (job instanceof ShootJob shoot) {
            return advanceShoot(shoot, statuses);
        }
        if (job instanceof SpinJob spin) {
            if (subsystems.shooter().isReady()) {
                statuses.add(RequestStatus.done(spin.id()));
                return true;
            }
            statuses.add(active(spin.id(), 0.0, "spinning up"));
            return false;
        }
        MotionJob motion = (MotionJob) job;
        if (!motion.started) {
            motion.started = true;
            statuses.add(active(motion.id(), 0.0, motion.note));
            return false;
        }
        if (subsystems.drive().pathDone()) {
            statuses.add(RequestStatus.done(motion.id()));
            return true;
        }
        statuses.add(active(motion.id(), 0.0, motion.note));
        return false;
    }

    /** Reports whether the underlying subsystem has completed this job. */
    public boolean isDone(Job job) {
        if (job instanceof ShootJob shoot) {
            return shoot.remaining <= 0 && !subsystems.shooter().isFeeding();
        }
        if (job instanceof SpinJob) {
            return subsystems.shooter().isReady();
        }
        MotionJob motion = (MotionJob) job;
        return motion.started && subsystems.drive().pathDone();
    }

    public void cancel(Job job, String note, List<RequestStatus> statuses) {
        if (job instanceof MotionJob motion) {
            subsystems.drive().stop();
            statuses.add(RequestStatus.rejected(motion.id(), note));
        } else if (job instanceof ShootJob shoot) {
            subsystems.shooter().spinDown();
            statuses.add(RequestStatus.rejected(shoot.id(), note));
        } else if (job instanceof SpinJob spin) {
            subsystems.shooter().spinDown();
            statuses.add(RequestStatus.rejected(spin.id(), note));
        }
    }

    public static boolean isDrive(RequestType type) {
        return type == RequestType.GOTO || type == RequestType.PATH || type == RequestType.TURN_TO;
    }

    private static boolean isShooter(RequestType type) {
        return type == RequestType.SHOOT || type == RequestType.SPIN_UP;
    }

    private Job startPath(Request request, List<RequestStatus> statuses) {
        if (request.path() == null) {
            statuses.add(RequestStatus.rejected(request.id(), "PATH requires a path payload"));
            return null;
        }
        subsystems.drive().follow(request.path());
        return new MotionJob(request.id(), "following");
    }

    private Job startTurn(Request request, List<RequestStatus> statuses) {
        double heading = request.param(0, Double.NaN);
        if (!Double.isFinite(heading)) {
            statuses.add(RequestStatus.rejected(request.id(), "TURN_TO requires a finite heading"));
            return null;
        }
        subsystems.drive().turnTo(heading);
        return new MotionJob(request.id(), "turning");
    }

    private Job startGoto(Request request, List<RequestStatus> statuses) {
        if (request.params().length < 3) {
            statuses.add(RequestStatus.rejected(request.id(), "GOTO requires x, y, heading"));
            return null;
        }
        Pose target = new Pose(request.param(0, 0), request.param(1, 0), request.param(2, 0));
        subsystems.drive().follow(PathRequest.goTo(target, PathRequest.Constraints.defaults()));
        return new MotionJob(request.id(), "following");
    }

    private Job startShoot(Request request, List<RequestStatus> statuses) {
        if (request.params().length < 1
                || !Double.isFinite(request.param(0, Double.NaN))
                || request.param(0, 0.0) <= 0.0
                || request.param(0, 0.0) != Math.rint(request.param(0, 0.0))) {
            statuses.add(RequestStatus.rejected(
                    request.id(), "SHOOT requires a positive integer count"));
            return null;
        }
        int count = (int) request.param(0, 0.0);
        if (count > RobotConstants.SHOT_MAX_COUNT) {
            statuses.add(RequestStatus.rejected(request.id(),
                    "SHOOT count must be 1.." + RobotConstants.SHOT_MAX_COUNT));
            return null;
        }
        boolean usePreset = profile == MechanismProfile.REAL || presetExplicit;
        double rpm = request.params().length >= 2
                ? request.param(1, Double.NaN)
                : usePreset ? preset.rpm()
                : ShooterCalibration.calibratedRpm(subsystems.drive().pose());
        if (!Double.isFinite(rpm) || rpm <= 0.0) {
            statuses.add(RequestStatus.rejected(request.id(),
                    "SHOOT requires a positive finite rpm"));
            return null;
        }
        ShootJob job = new ShootJob(request.id(), count, rpm, usePreset,
                request.params().length >= 2, nowMs);
        if (usePreset) {
            // Review B08 major 2: a preset shot never feeds past a rejected turret aim.
            String rejected = latch(job);
            if (rejected != null) {
                statuses.add(RequestStatus.rejected(request.id(), rejected));
                return null;
            }
        }
        subsystems.shooter().spinUp(job.rpm);
        return job;
    }

    /** A preset shot owns the turret target: a TURRET_AIM retargets it away. */
    public boolean ownsTurretTarget(Job job) {
        return job instanceof ShootJob shoot && shoot.gated;
    }

    private Job setPreset(Request request, List<RequestStatus> statuses) {
        double[] p = request.params();
        String problem = p.length < 3 ? "SET_SHOT_PRESET requires rpm, hoodDeg, turretRad"
                : ShotPreset.validate(p[0], p[1], p[2]);
        if (problem != null) {
            statuses.add(RequestStatus.rejected(request.id(), problem));
            return null;
        }
        preset = new ShotPreset(p[0], p[1], p[2]);
        presetExplicit = true;
        statuses.add(RequestStatus.done(request.id()));
        return null;
    }

    private Job startSpin(Request request, List<RequestStatus> statuses) {
        double rpm = request.param(0, 1.0);
        if (!Double.isFinite(rpm) || rpm <= 0.0) {
            statuses.add(RequestStatus.rejected(request.id(), "SPIN_UP requires positive rpm"));
            return null;
        }
        subsystems.shooter().spinUp(rpm);
        return new SpinJob(request.id());
    }

    private Job startIntake(Request request, List<RequestStatus> statuses) {
        if (request.type() == RequestType.INTAKE_OFF
                || (request.type() == RequestType.INTAKE
                && request.param(0, 0.0) == 0.0)) {
            subsystems.intake().stop();
        } else {
            subsystems.intake().run(request.param(0, 1.0));
        }
        statuses.add(RequestStatus.done(request.id()));
        return null;
    }

    private Job startTurretAim(Request request, List<RequestStatus> statuses) {
        if (request.params().length < 2
                || !Double.isFinite(request.param(0, Double.NaN))
                || !Double.isFinite(request.param(1, Double.NaN))) {
            statuses.add(RequestStatus.rejected(request.id(), "TURRET_AIM requires finite x, y"));
            return null;
        }
        subsystems.turret().aimAt(request.param(0, 0.0), request.param(1, 0.0));
        statuses.add(RequestStatus.done(request.id()));
        return null;
    }

    private Job startResetPose(Request request, List<RequestStatus> statuses) {
        if (request.params().length < 3
                || !Double.isFinite(request.param(0, Double.NaN))
                || !Double.isFinite(request.param(1, Double.NaN))
                || !Double.isFinite(request.param(2, Double.NaN))) {
            statuses.add(RequestStatus.rejected(
                    request.id(), "RESET_POSE requires finite x, y, heading"));
            return null;
        }
        subsystems.drive().resetPose(new Pose(request.param(0, 0.0), request.param(1, 0.0),
                request.param(2, 0.0)));
        statuses.add(RequestStatus.done(request.id()));
        return null;
    }

    private boolean advanceShoot(ShootJob shoot, List<RequestStatus> statuses) {
        var shooter = subsystems.shooter();
        if (shooter.isFeeding()) {
            if (!shoot.stopping) {
                shooter.spinUp(shoot.rpm);
            }
            statuses.add(active(shoot.id(), shoot.progress(), "feeding"));
            return false;
        }
        if (shoot.remaining <= 0) {
            if (shoot.stopping) {
                // Archive: releasing RT (FINISHING_PULSE -> IDLE) disables the shooter.
                shooter.spinDown();
            }
            statuses.add(shoot.stopped
                    ? new RequestStatus(shoot.id(), RequestStatus.State.DONE, shoot.progress(),
                    "stopped")
                    : RequestStatus.done(shoot.id()));
            return true;
        }
        if (shoot.gated && !preset.equals(shoot.latched)) {
            // Like cplx1: a SET_SHOT_PRESET during a shot applies from the next pulse.
            String rejected = latch(shoot);
            if (rejected != null) {
                return fail(shoot, rejected, statuses);
            }
        }
        shooter.spinUp(shoot.rpm);
        if (feedHeld) {
            shoot.prepareSinceMs = -1;
            statuses.add(active(shoot.id(), shoot.progress(), "mechanism recovery"));
            return false;
        }
        String blocked = shoot.gated ? gatedBlockReason(shoot)
                : shooter.isReady() ? null : "spinning up";
        if (blocked != null && blocked.startsWith("turret: ")) {
            return fail(shoot, blocked, statuses);
        }
        if (blocked != null) {
            if (shoot.gated) {
                // Same bound as cplx1: the prepare timeout starts once the turret is up, or
                // after one archive calibration window at the latest.
                if (shoot.prepareSinceMs < 0 && (subsystems.turret().aimStatus()
                        != ITurret.AimResult.NOT_INITIALIZED
                        || nowMs - shoot.beganMs >= RobotConstants.SHOT_TURRET_STARTUP_BOUND_MS)) {
                    shoot.prepareSinceMs = nowMs;
                }
                if (shoot.prepareSinceMs >= 0
                        && nowMs - shoot.prepareSinceMs > RobotConstants.SHOT_PREPARE_TIMEOUT_MS) {
                    return fail(shoot, "prepare timeout: " + blocked, statuses);
                }
            }
            statuses.add(active(shoot.id(), shoot.progress(), blocked));
            return false;
        }
        shooter.feed();
        if (shooter.isFeeding()) {
            shoot.remaining--;
            shoot.fired++;
            shoot.prepareSinceMs = -1;
            statuses.add(active(shoot.id(), shoot.progress(), "feeding"));
        } else {
            statuses.add(active(shoot.id(), shoot.progress(), "feed not accepted"));
        }
        return false;
    }

    /** Null when a preset shot may feed, otherwise the first blocking reason. */
    private String gatedBlockReason(ShootJob shoot) {
        ITurret turret = subsystems.turret();
        if (turret.aimStatus() != ITurret.AimResult.ACCEPTED) {
            // Re-aim every tick: a turret still starting (or recalibrating) takes the
            // latched preset angle as soon as it can.
            ITurret.AimResult aim = turret.aimRelative(shoot.latched.turretRad());
            if (aim != ITurret.AimResult.ACCEPTED && aim != ITurret.AimResult.NOT_INITIALIZED) {
                return "turret: " + aim;
            }
        }
        if (!subsystems.shooter().isReady()) {
            return "spinning up";
        }
        if (!subsystems.shooter().hoodSettled()) {
            return "hood settling";
        }
        if (!turret.onTarget()) {
            ITurret.AimResult aim = turret.aimStatus();
            return aim == ITurret.AimResult.ACCEPTED ? "aiming" : "aiming: " + aim;
        }
        return null;
    }

    /** Latches the current preset into a gated shot; a rejection note when the aim fails. */
    private String latch(ShootJob shoot) {
        shoot.latched = preset;
        if (!shoot.explicitRpm) {
            shoot.rpm = preset.rpm();
        }
        subsystems.shooter().setHoodAngleDeg(preset.hoodDeg());
        ITurret.AimResult aim = subsystems.turret().aimRelative(preset.turretRad());
        return aim == ITurret.AimResult.ACCEPTED || aim == ITurret.AimResult.NOT_INITIALIZED
                ? null : "turret: " + aim;
    }

    private boolean fail(ShootJob shoot, String note, List<RequestStatus> statuses) {
        subsystems.shooter().spinDown();
        statuses.add(new RequestStatus(shoot.id(), RequestStatus.State.FAILED,
                shoot.progress(), note));
        return true;
    }

    private static RequestStatus active(int id, double progress, String note) {
        return new RequestStatus(id, RequestStatus.State.ACTIVE, progress, note);
    }

    public abstract static class Job {
        private final int id;

        private Job(int id) {
            this.id = id;
        }

        public int id() {
            return id;
        }
    }

    private static final class ShootJob extends Job {
        private final int count;
        /** Preset shots wait for hood and turret like the cplx1 feed gates. */
        private final boolean gated;
        /** The request named its rpm; a preset change keeps it. */
        private final boolean explicitRpm;
        private final long beganMs;
        private double rpm;
        private ShotPreset latched;
        private int remaining;
        private int fired;
        private boolean stopping;
        private boolean stopped;
        /** Start of the current prepare wait, -1 while not waiting (cplx1 semantics). */
        private long prepareSinceMs = -1;

        private ShootJob(int id, int count, double rpm, boolean gated, boolean explicitRpm,
                         long beganMs) {
            super(id);
            this.count = count;
            this.remaining = count;
            this.rpm = rpm;
            this.gated = gated;
            this.explicitRpm = explicitRpm;
            this.beganMs = beganMs;
        }

        private double progress() {
            return fired / (double) count;
        }
    }

    private static final class SpinJob extends Job {
        private SpinJob(int id) {
            super(id);
        }
    }

    private static final class MotionJob extends Job {
        private final String note;
        private boolean started;

        private MotionJob(int id, String note) {
            super(id);
            this.note = note;
        }
    }
}
