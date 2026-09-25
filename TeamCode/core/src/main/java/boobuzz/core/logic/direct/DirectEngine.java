package boobuzz.core.logic.direct;

import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.logic.IRobotEngine;
import boobuzz.core.subsystem.Subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/** Wiring-only engine: stream arbitration, job ownership, and status draining. */
public final class DirectEngine implements IRobotEngine {

    private final Subsystems subsystems;
    private final DirectMap map;
    private DirectMap.Job driveJob;
    /**
     * True while the last DONE drive request's end hold is running. Like the archive
     * and simple-code, DONE does not stop the follower: it keeps holding the end pose
     * until the next drive request, manual drive, a cancel of {@link #holdingDriveId},
     * CANCEL_ALL or RESET_POSE.
     */
    private boolean holdingDrive;
    private int holdingDriveId;
    private DirectMap.Job shooterJob;
    private List<RequestStatus> pendingStatuses = Collections.emptyList();
    private RobotAction action = RobotAction.zero();
    /** Actuator owner, separate from terminal request IDs. */
    private Integer intakeOwnerId;

    public DirectEngine(Subsystems subsystems) {
        this.subsystems = Objects.requireNonNull(subsystems, "subsystems");
        this.map = new DirectMap(subsystems);
    }

    @Override
    public String name() {
        return "direct";
    }

    @Override
    public WorldSnapshot sense(RobotState state) {
        subsystems.observe(state);
        return new WorldSnapshot(
                state.t(), subsystems.drive().pose(), state.yaw(), state.voltage());
    }

    @Override
    public void act(RequestBatch batch) {
        if (batch == null) {
            batch = RequestBatch.idle();
        }
        List<RequestStatus> statuses = new ArrayList<>();
        boolean cancelAll = containsCancelAll(batch.cancels());
        if (cancelAll) {
            cancelAll(statuses);
        }
        boolean driveRequest = batch.requests().stream()
                .anyMatch(request -> DirectMap.isDrive(request.type()));

        if (batch.stream().manualDrive()) {
            cancelDrive(statuses, "overridden by manual drive");
            holdingDrive = false;   // manual() replaces the follower's hold
            subsystems.drive().manual(batch.stream().vx(), batch.stream().vy(),
                    batch.stream().omega());
        }

        for (int id : batch.cancels()) {
            if (id == RequestBatch.CANCEL_ALL) {
                continue;
            }
            cancel(id, statuses);
            if (holdingDrive && holdingDriveId == id) {
                releaseDriveHold();
            }
            if (intakeOwnerId != null && intakeOwnerId == id) {
                subsystems.intake().stop();
                intakeOwnerId = null;
            }
        }

        for (Request request : batch.requests()) {
            if (batch.stream().manualDrive() && DirectMap.isDrive(request.type())) {
                statuses.add(RequestStatus.rejected(request.id(), "overridden by manual drive"));
                continue;
            }
            if (request.type() == boobuzz.core.contract.RequestType.RESET_POSE
                    && driveJob != null) {
                map.cancel(driveJob, "reset pose", statuses);
                driveJob = null;
            }
            if (request.type() == boobuzz.core.contract.RequestType.RESET_POSE) {
                holdingDrive = false;   // resetPose() drops the follower's request
            }
            DirectMap.Job job = map.start(request, driveJob != null, shooterJob != null, statuses);
            if (job == null) {
                if (isIntake(request)) {
                    if (request.type() == boobuzz.core.contract.RequestType.INTAKE_OFF
                            || (request.type() == boobuzz.core.contract.RequestType.INTAKE
                            && request.param(0, 0.0) == 0.0)) {
                        intakeOwnerId = null;
                    } else {
                        intakeOwnerId = request.id();
                    }
                }
                continue;
            }
            if (DirectMap.isDrive(request.type())) {
                driveJob = job;
                holdingDrive = false;   // the new request replaces the hold
            } else if (isShooter(request)) {
                shooterJob = job;
            }
        }

        if (!batch.stream().manualDrive() && !driveRequest && driveJob == null
                && !holdingDrive) {
            subsystems.drive().stop();
        }

        if (driveJob != null && map.advance(driveJob, statuses)) {
            holdingDrive = true;   // DONE: keep the end hold running
            holdingDriveId = driveJob.id();
            driveJob = null;
        }
        if (shooterJob != null && map.advance(shooterJob, statuses)) {
            shooterJob = null;
        }

        pendingStatuses = Collections.unmodifiableList(new ArrayList<>(statuses));
        action = subsystems.update();
    }

    @Override
    public RobotAction action() {
        return action;
    }

    @Override
    public List<RequestStatus> drainStatuses() {
        List<RequestStatus> statuses = pendingStatuses;
        pendingStatuses = Collections.emptyList();
        return statuses;
    }

    public Subsystems subsystems() {
        return subsystems;
    }

    private void cancel(int id, List<RequestStatus> statuses) {
        if (driveJob != null && driveJob.id() == id) {
            map.cancel(driveJob, "cancelled", statuses);
            driveJob = null;
        } else if (shooterJob != null && shooterJob.id() == id) {
            map.cancel(shooterJob, "cancelled", statuses);
            shooterJob = null;
        }
    }

    private void cancelDrive(List<RequestStatus> statuses, String note) {
        if (driveJob != null) {
            map.cancel(driveJob, note, statuses);
            driveJob = null;
        }
    }

    private void cancelAll(List<RequestStatus> statuses) {
        if (driveJob != null) {
            map.cancel(driveJob, "engine switch", statuses);
            driveJob = null;
        }
        if (shooterJob != null) {
            map.cancel(shooterJob, "engine switch", statuses);
            shooterJob = null;
        }
        holdingDrive = false;
        subsystems.drive().stop();
        subsystems.shooter().spinDown();
        subsystems.intake().stop();
        intakeOwnerId = null;
        subsystems.turret().hold();
    }

    /** Stops the end hold of a DONE drive request, if one is running. */
    private void releaseDriveHold() {
        if (!holdingDrive) {
            return;
        }
        holdingDrive = false;
        subsystems.drive().stop();
    }

    private static boolean containsCancelAll(int[] cancels) {
        for (int id : cancels) {
            if (id == RequestBatch.CANCEL_ALL) {
                return true;
            }
        }
        return false;
    }

    private static boolean isShooter(Request request) {
        return request.type() == boobuzz.core.contract.RequestType.SHOOT
                || request.type() == boobuzz.core.contract.RequestType.SPIN_UP;
    }

    private static boolean isIntake(Request request) {
        return request.type() == boobuzz.core.contract.RequestType.INTAKE
                || request.type() == boobuzz.core.contract.RequestType.INTAKE_ON
                || request.type() == boobuzz.core.contract.RequestType.INTAKE_OFF;
    }
}
