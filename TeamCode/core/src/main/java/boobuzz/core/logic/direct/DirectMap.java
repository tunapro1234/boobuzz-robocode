package boobuzz.core.logic.direct;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestType;
import boobuzz.core.subsystem.Subsystems;

import com.pedropathing.math.Pose;

import java.util.List;

/**
 * Direct request-to-subsystem map. To add a request type: (1) RequestType enum,
 * (2) one case here, (3) done.
 */
public final class DirectMap {

    private final Subsystems subsystems;

    public DirectMap(Subsystems subsystems) {
        this.subsystems = subsystems;
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
        int count = (int) Math.round(request.param(0, 1.0));
        double rpm = request.param(1, 1.0);
        if (count <= 0 || !Double.isFinite(rpm) || rpm <= 0.0) {
            statuses.add(RequestStatus.rejected(request.id(), "SHOOT requires positive count and rpm"));
            return null;
        }
        subsystems.shooter().spinUp(rpm);
        return new ShootJob(request.id(), count, rpm);
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

    private boolean advanceShoot(ShootJob shoot, List<RequestStatus> statuses) {
        var shooter = subsystems.shooter();
        shooter.spinUp(shoot.rpm);
        if (shooter.isFeeding()) {
            statuses.add(active(shoot.id(), 0.5, "feeding"));
        } else if (!shooter.isReady()) {
            statuses.add(active(shoot.id(), 0.0, "spinning up"));
        } else if (shoot.remaining > 0) {
            shooter.feed();
            if (shooter.isFeeding()) {
                shoot.remaining--;
                statuses.add(active(shoot.id(), 0.5, "feeding"));
            } else {
                statuses.add(active(shoot.id(), 0.0, "feed not accepted"));
            }
        } else {
            statuses.add(RequestStatus.done(shoot.id()));
            return true;
        }
        return false;
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
        private int remaining;

        private ShootJob(int id, int remaining, double rpm) {
            super(id);
            this.remaining = remaining;
            this.rpm = rpm;
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
