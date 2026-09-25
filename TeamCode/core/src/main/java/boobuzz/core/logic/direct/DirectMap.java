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
        ShootJob job = new ShootJob(request.id(), count, rpm, usePreset, preset.turretRad());
        if (usePreset) {
            // Review B08 major 2: a preset shot never feeds past a rejected turret aim.
            ITurret.AimResult aim = subsystems.turret().aimRelative(preset.turretRad());
            if (aim != ITurret.AimResult.ACCEPTED && aim != ITurret.AimResult.NOT_INITIALIZED) {
                statuses.add(RequestStatus.rejected(request.id(), "turret: " + aim));
                return null;
            }
            job.aimSinceMs = aim == ITurret.AimResult.ACCEPTED ? -1 : nowMs;
            subsystems.shooter().setHoodAngleDeg(preset.hoodDeg());
        }
        subsystems.shooter().spinUp(rpm);
        return job;
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
        if (!shoot.stopping) {
            shooter.spinUp(shoot.rpm);
        }
        String aimProblem = shoot.gated && !shooter.isFeeding() && shoot.remaining > 0
                ? reaim(shoot) : null;
        if (aimProblem != null && !aimProblem.isEmpty()) {
            shooter.spinDown();
            statuses.add(new RequestStatus(shoot.id(), RequestStatus.State.FAILED, 0.0,
                    aimProblem));
            return true;
        }
        if (shooter.isFeeding()) {
            statuses.add(active(shoot.id(), 0.5, "feeding"));
        } else if (shoot.remaining > 0 && aimProblem != null) {
            statuses.add(active(shoot.id(), 0.0, "turret starting"));
        } else if (shoot.remaining > 0 && !shooter.isReady()) {
            statuses.add(active(shoot.id(), 0.0, "spinning up"));
        } else if (shoot.remaining > 0 && shoot.gated && !shooter.hoodSettled()) {
            statuses.add(active(shoot.id(), 0.0, "hood settling"));
        } else if (shoot.remaining > 0 && shoot.gated && !subsystems.turret().onTarget()) {
            statuses.add(active(shoot.id(), 0.0, "aiming"));
        } else if (shoot.remaining > 0 && feedHeld) {
            statuses.add(active(shoot.id(), 0.0, "mechanism recovery"));
        } else if (shoot.remaining > 0) {
            shooter.feed();
            if (shooter.isFeeding()) {
                shoot.remaining--;
                statuses.add(active(shoot.id(), 0.5, "feeding"));
            } else {
                statuses.add(active(shoot.id(), 0.0, "feed not accepted"));
            }
        } else {
            if (shoot.stopping) {
                shooter.spinDown();
            }
            statuses.add(RequestStatus.done(shoot.id()));
            return true;
        }
        return false;
    }

    /**
     * Keeps a gated shot's turret aim alive: null while the aim is accepted, "" while the
     * turret is still starting (re-aimed every tick), a failure note when the aim is
     * rejected or startup outlasts one archive calibration window.
     */
    private String reaim(ShootJob shoot) {
        ITurret turret = subsystems.turret();
        if (turret.aimStatus() == ITurret.AimResult.ACCEPTED) {
            shoot.aimSinceMs = -1;
            return null;
        }
        ITurret.AimResult aim = turret.aimRelative(shoot.turretRad);
        if (aim == ITurret.AimResult.ACCEPTED) {
            shoot.aimSinceMs = -1;
            return null;
        }
        if (aim != ITurret.AimResult.NOT_INITIALIZED) {
            return "turret: " + aim;
        }
        if (shoot.aimSinceMs < 0) {
            shoot.aimSinceMs = nowMs;
        }
        if (nowMs - shoot.aimSinceMs >= RobotConstants.SHOT_TURRET_STARTUP_BOUND_MS) {
            return "turret startup timeout: " + aim;
        }
        return "";
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
        private final double rpm;
        /** Preset shots wait for hood and turret like the cplx1 feed gates. */
        private final boolean gated;
        private final double turretRad;
        private int remaining;
        private boolean stopping;
        /** Start of the current NOT_INITIALIZED wait, -1 while the aim is accepted. */
        private long aimSinceMs = -1;

        private ShootJob(int id, int remaining, double rpm, boolean gated, double turretRad) {
            super(id);
            this.remaining = remaining;
            this.rpm = rpm;
            this.gated = gated;
            this.turretRad = turretRad;
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
