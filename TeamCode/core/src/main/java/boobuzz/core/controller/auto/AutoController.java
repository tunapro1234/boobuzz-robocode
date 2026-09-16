package boobuzz.core.controller.auto;

import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.Intent;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestStatus;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** Controller that advances an {@link AutoSequence} from engine statuses and HAL time. */
public final class AutoController implements boobuzz.core.controller.Controller {

    private enum Phase { READY, MOTION, REQUEST, WAIT, INTAKE_TIME }

    private final AutoSequence sequence;
    private int nextRequestId = 1;
    private Phase phase = Phase.READY;
    private int primaryRequestId = -1;
    private List<Integer> attachedRequestIds = List.of();
    private long timerStartMs;
    private RequestStatus failure;

    public AutoController(AutoSequence sequence) {
        this.sequence = Objects.requireNonNull(sequence, "sequence");
    }

    @Override
    public Intent decide(Feedback feedback) {
        long now = feedback == null ? timerStartMs : feedback.t();
        List<RequestStatus> statuses = feedback == null ? List.of() : feedback.statuses();

        if (failure != null || sequence.isDone()) {
            return Intent.idle();
        }
        if (hasFailure(statuses)) {
            return Intent.idle();
        }

        if (phase == Phase.WAIT) {
            AutoStep.Wait wait = (AutoStep.Wait) sequence.current();
            if (elapsed(now, timerStartMs) >= secondsToMillis(wait.seconds())) {
                advance();
            }
            return Intent.idle();
        }

        if (phase == Phase.INTAKE_TIME) {
            if (elapsed(now, timerStartMs)
                    >= secondsToMillis(((AutoStep.Intake) sequence.current()).seconds())) {
                int offId = allocateId();
                primaryRequestId = offId;
                attachedRequestIds = List.of();
                phase = Phase.REQUEST;
                return intent(Request.intakeOff(offId));
            }
            return Intent.idle();
        }

        if (phase == Phase.MOTION || phase == Phase.REQUEST) {
            RequestStatus primary = status(statuses, primaryRequestId);
            if (primary != null && primary.state() == RequestStatus.State.DONE) {
                AutoStep current = sequence.current();
                if (phase == Phase.MOTION && current instanceof AutoStep.Path path
                        && path.intake()) {
                    int offId = allocateId();
                    primaryRequestId = offId;
                    attachedRequestIds = List.of();
                    phase = Phase.REQUEST;
                    return intent(Request.intakeOff(offId));
                }
                advance();
            }
            return Intent.idle();
        }

        AutoStep step = sequence.current();
        if (step instanceof AutoStep.Path path) {
            List<Request> requests = new ArrayList<>(3);
            int pathId = allocateId();
            requests.add(Request.path(pathId, path.request()));
            List<Integer> attached = new ArrayList<>(2);
            if (path.intake()) {
                int intakeId = allocateId();
                requests.add(Request.intakeOn(intakeId, path.intakePower()));
                attached.add(intakeId);
            }
            if (path.shooterWarmupRpm() != null) {
                int warmupId = allocateId();
                requests.add(Request.spinUp(warmupId, path.shooterWarmupRpm()));
                attached.add(warmupId);
            }
            primaryRequestId = pathId;
            attachedRequestIds = List.copyOf(attached);
            phase = Phase.MOTION;
            return intent(requests);
        }
        if (step instanceof AutoStep.Turn turn) {
            int id = allocateId();
            primaryRequestId = id;
            attachedRequestIds = List.of();
            phase = Phase.MOTION;
            return intent(Request.turnTo(id, turn.headingRad()));
        }
        if (step instanceof AutoStep.Shoot shoot) {
            int id = allocateId();
            primaryRequestId = id;
            attachedRequestIds = List.of();
            phase = Phase.REQUEST;
            return intent(Request.shoot(id, shoot.count(), shoot.rpm()));
        }
        if (step instanceof AutoStep.SpinUp spinUp) {
            int id = allocateId();
            primaryRequestId = id;
            attachedRequestIds = List.of();
            phase = Phase.REQUEST;
            return intent(Request.spinUp(id, spinUp.rpm()));
        }
        if (step instanceof AutoStep.Intake intake) {
            int id = allocateId();
            primaryRequestId = id;
            attachedRequestIds = List.of();
            timerStartMs = now;
            phase = Phase.INTAKE_TIME;
            return intent(Request.intakeOn(id, intake.power()));
        }
        AutoStep.Wait wait = (AutoStep.Wait) step;
        timerStartMs = now;
        phase = Phase.WAIT;
        if (wait.seconds() == 0.0) {
            advance();
        }
        return Intent.idle();
    }

    public AutoSequence sequence() {
        return sequence;
    }

    public boolean isDone() {
        return sequence.isDone();
    }

    public boolean isFinished() {
        return isDone() || isFailed();
    }

    public boolean isFailed() {
        return failure != null;
    }

    public RequestStatus failure() {
        return failure;
    }

    public String failureNote() {
        return failure == null ? "" : failure.note();
    }

    private Intent intent(Request request) {
        return intent(List.of(request));
    }

    private Intent intent(List<Request> requests) {
        return new Intent(Drive.HOLD, requests, new int[0]);
    }

    private boolean hasFailure(List<RequestStatus> statuses) {
        for (int id : activeRequestIds()) {
            RequestStatus status = status(statuses, id);
            if (status != null && (status.state() == RequestStatus.State.REJECTED
                    || status.state() == RequestStatus.State.FAILED)) {
                failure = status;
                return true;
            }
        }
        return false;
    }

    private List<Integer> activeRequestIds() {
        if (primaryRequestId < 0) {
            return List.of();
        }
        List<Integer> ids = new ArrayList<>(attachedRequestIds.size() + 1);
        ids.add(primaryRequestId);
        ids.addAll(attachedRequestIds);
        return ids;
    }

    private static RequestStatus status(List<RequestStatus> statuses, int id) {
        for (RequestStatus status : statuses) {
            if (status.id() == id) {
                return status;
            }
        }
        return null;
    }

    private int allocateId() {
        return nextRequestId++;
    }

    private void advance() {
        sequence.advance();
        primaryRequestId = -1;
        attachedRequestIds = List.of();
        phase = Phase.READY;
    }

    private static long elapsed(long now, long start) {
        return Math.max(0L, now - start);
    }

    private static long secondsToMillis(double seconds) {
        return Math.round(seconds * 1000.0);
    }
}
