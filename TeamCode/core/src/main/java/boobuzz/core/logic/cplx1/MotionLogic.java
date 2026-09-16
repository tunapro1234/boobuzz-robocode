package boobuzz.core.logic.cplx1;

import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.contract.RequestType;
import boobuzz.core.subsystem.IDrive;

import com.pedropathing.math.Pose;

import java.util.List;
import java.util.Objects;

/** Owns drive arbitration between the per-tick stream and edge-triggered requests. */
public final class MotionLogic {

    private final IDrive drive;
    private MotionJob active;

    public MotionLogic(IDrive drive) {
        this.drive = Objects.requireNonNull(drive, "drive");
    }

    public void act(RequestStream stream, List<Request> requests, int[] cancels,
                    List<RequestStatus> statuses) {
        stream = stream == null ? RequestStream.idle() : stream;
        boolean driveRequest = requests.stream().anyMatch(request -> isDrive(request.type()));

        if (stream.manualDrive()) {
            cancelActive("overridden by manual drive", statuses);
            drive.manual(stream.vx(), stream.vy(), stream.omega());
        }
        for (int id : cancels == null ? new int[0] : cancels) {
            if (id == boobuzz.core.contract.RequestBatch.CANCEL_ALL) {
                cancelActive("engine switch", statuses);
                continue;
            }
            if (active != null && active.id == id) {
                cancelActive("cancelled", statuses);
            }
        }

        for (Request request : requests) {
            if (!isDrive(request.type())) {
                continue;
            }
            if (stream.manualDrive()) {
                statuses.add(RequestStatus.rejected(request.id(), "overridden by manual drive"));
            } else if (active != null) {
                statuses.add(RequestStatus.rejected(request.id(), "drive already has a request"));
            } else {
                active = start(request, statuses);
            }
        }

        if (!stream.manualDrive() && !driveRequest && active == null) {
            drive.stop();
        }
        advance(statuses);
    }

    public boolean hasActiveRequest() {
        return active != null;
    }

    public int activeRequestId() {
        return active == null ? -1 : active.id;
    }

    public void cancelAll(List<RequestStatus> statuses) {
        cancelActive("engine switch", statuses);
        drive.stop();
    }

    public void resetPose(Pose pose, List<RequestStatus> statuses) {
        Objects.requireNonNull(pose, "pose");
        cancelActive("reset pose", statuses);
        drive.resetPose(pose);
    }

    private MotionJob start(Request request, List<RequestStatus> statuses) {
        switch (request.type()) {
            case PATH -> {
                if (request.path() == null) {
                    statuses.add(RequestStatus.rejected(request.id(), "PATH requires a path payload"));
                    return null;
                }
                drive.follow(request.path());
                return new MotionJob(request.id(), "following");
            }
            case GOTO -> {
                if (request.params().length < 3) {
                    statuses.add(RequestStatus.rejected(request.id(), "GOTO requires x, y, heading"));
                    return null;
                }
                Pose target = new Pose(request.param(0, 0), request.param(1, 0),
                        request.param(2, 0));
                drive.follow(PathRequest.goTo(target, PathRequest.Constraints.defaults()));
                return new MotionJob(request.id(), "following");
            }
            case TURN_TO -> {
                double heading = request.param(0, Double.NaN);
                if (!Double.isFinite(heading)) {
                    statuses.add(RequestStatus.rejected(
                            request.id(), "TURN_TO requires a finite heading"));
                    return null;
                }
                drive.turnTo(heading);
                return new MotionJob(request.id(), "turning");
            }
            default -> throw new AssertionError("not a drive request: " + request.type());
        }
    }

    private void advance(List<RequestStatus> statuses) {
        if (active == null) {
            return;
        }
        if (!active.started) {
            active.started = true;
            statuses.add(active(active.id, 0.0, active.note));
        } else if (drive.pathDone()) {
            statuses.add(RequestStatus.done(active.id));
            active = null;
        } else {
            statuses.add(active(active.id, 0.0, active.note));
        }
    }

    private void cancelActive(String note, List<RequestStatus> statuses) {
        if (active == null) {
            return;
        }
        statuses.add(RequestStatus.rejected(active.id, note));
        active = null;
        drive.stop();
    }

    private static boolean isDrive(RequestType type) {
        return type == RequestType.GOTO || type == RequestType.PATH || type == RequestType.TURN_TO;
    }

    private static RequestStatus active(int id, double progress, String note) {
        return new RequestStatus(id, RequestStatus.State.ACTIVE, progress, note);
    }

    private static final class MotionJob {
        private final int id;
        private final String note;
        private boolean started;

        private MotionJob(int id, String note) {
            this.id = id;
            this.note = note;
        }
    }
}
