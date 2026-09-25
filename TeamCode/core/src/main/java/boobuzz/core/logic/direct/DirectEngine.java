package boobuzz.core.logic.direct;

import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestType;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.logic.IRobotEngine;
import boobuzz.core.logic.MechanismProfile;
import boobuzz.core.logic.shot.MechanismRecovery;
import boobuzz.core.subsystem.Subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/** Wiring-only engine: stream arbitration, job ownership, and status draining. */
public final class DirectEngine implements IRobotEngine {

    private final Subsystems subsystems;
    private final DirectMap map;
    private final MechanismRecovery recovery;
    private DirectMap.Job driveJob;
    private DirectMap.Job shooterJob;
    private List<RequestStatus> pendingStatuses = Collections.emptyList();
    private RobotAction action = RobotAction.zero();
    /** Actuator owner, separate from terminal request IDs. */
    private Integer intakeOwnerId;
    /** Current manual intake demand; restored when mechanism recovery releases the intake. */
    private double manualIntakePower;
    private boolean recoveryOwnedIntake;

    public DirectEngine(Subsystems subsystems) {
        this(subsystems, MechanismProfile.STUB);
    }

    public DirectEngine(Subsystems subsystems, MechanismProfile profile) {
        this.subsystems = Objects.requireNonNull(subsystems, "subsystems");
        this.map = new DirectMap(subsystems, profile);
        this.recovery = new MechanismRecovery(subsystems.shooter());
    }

    @Override
    public String name() {
        return "direct";
    }

    @Override
    public WorldSnapshot sense(RobotState state) {
        subsystems.observe(state);
        subsystems.turret().setRobotPose(subsystems.drive().pose());
        recovery.observe(state.t());
        map.observe(state.t());
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
        List<Request> requests = RequestBatch.withoutCancelled(batch, statuses);
        boolean driveRequest = requests.stream()
                .anyMatch(request -> DirectMap.isDrive(request.type()));

        if (batch.stream().manualDrive()) {
            cancelDrive(statuses, "overridden by manual drive");
            subsystems.drive().manual(batch.stream().vx(), batch.stream().vy(),
                    batch.stream().omega());
        }

        for (int id : batch.cancels()) {
            if (id == RequestBatch.CANCEL_ALL) {
                continue;
            }
            cancel(id, statuses);
            recovery.cancel(id, statuses);
            if (intakeOwnerId != null && intakeOwnerId == id) {
                subsystems.intake().stop();
                intakeOwnerId = null;
                manualIntakePower = 0.0;
            }
        }

        for (Request request : requests) {
            if (batch.stream().manualDrive() && DirectMap.isDrive(request.type())) {
                statuses.add(RequestStatus.rejected(request.id(), "overridden by manual drive"));
                continue;
            }
            if (request.type() == boobuzz.core.contract.RequestType.RESET_POSE
                    && driveJob != null) {
                map.cancel(driveJob, "reset pose", statuses);
                driveJob = null;
            }
            if (request.type() == RequestType.TURRET_AIM && map.ownsTurretTarget(shooterJob)) {
                // Like cplx1: a field aim takes the turret away from a preset shot.
                map.cancel(shooterJob, "turret retargeted", statuses);
                shooterJob = null;
            }
            if (request.type() == RequestType.STOP_SHOOTING) {
                if (shooterJob != null || !recovery.blocksShooter()) {
                    if (map.stopShooting(shooterJob, statuses)) {
                        shooterJob = null;
                    }
                }
                statuses.add(RequestStatus.done(request.id()));
                continue;
            }
            if (request.type() == RequestType.MECHANISM_RECOVERY) {
                handleRecovery(request, statuses);
                continue;
            }
            if (isShooter(request) && recovery.blocksShooter()) {
                statuses.add(RequestStatus.rejected(request.id(),
                        "mechanism recovery owns the flywheel"));
                continue;
            }
            DirectMap.Job job = map.start(request, driveJob != null, shooterJob != null, statuses);
            if (job == null) {
                if (isIntake(request)) {
                    if (request.type() == boobuzz.core.contract.RequestType.INTAKE_OFF
                            || (request.type() == boobuzz.core.contract.RequestType.INTAKE
                            && request.param(0, 0.0) == 0.0)) {
                        intakeOwnerId = null;
                        manualIntakePower = 0.0;
                    } else {
                        intakeOwnerId = request.id();
                        double power = request.param(0, 1.0);
                        manualIntakePower = Double.isFinite(power) ? power : 0.0;
                    }
                }
                continue;
            }
            if (DirectMap.isDrive(request.type())) {
                driveJob = job;
            } else if (isShooter(request)) {
                shooterJob = job;
            }
        }

        if (!batch.stream().manualDrive() && !driveRequest && driveJob == null) {
            subsystems.drive().stop();
        }

        if (driveJob != null && map.advance(driveJob, statuses)) {
            driveJob = null;
        }
        map.holdFeed(recovery.holdsFeed());
        if (shooterJob != null && map.advance(shooterJob, statuses)) {
            shooterJob = null;
        }
        recovery.update(statuses);
        applyRecoveryIntake();

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
        subsystems.drive().stop();
        subsystems.shooter().spinDown();
        subsystems.intake().stop();
        intakeOwnerId = null;
        manualIntakePower = 0.0;
        recovery.cancelAll(statuses);
        recoveryOwnedIntake = false;
        subsystems.turret().disable();
    }

    private void handleRecovery(Request request, List<RequestStatus> statuses) {
        int mode = MechanismRecovery.mode(request);
        if (mode < 0) {
            statuses.add(RequestStatus.rejected(request.id(),
                    "MECHANISM_RECOVERY requires mode 0, 1 or 2"));
            return;
        }
        if (mode == 2 && shooterJob != null) {
            // The jam clear drives the flywheel open loop: no shooter request survives it.
            map.cancel(shooterJob, "jam clear", statuses);
            shooterJob = null;
        }
        recovery.request(request.id(), mode, statuses);
    }

    /** Recovery overrides the intake while it owns it; release restores the manual demand. */
    private void applyRecoveryIntake() {
        if (recovery.ownsIntake()) {
            subsystems.intake().run(recovery.intakePower());
            recoveryOwnedIntake = true;
        } else if (recoveryOwnedIntake) {
            recoveryOwnedIntake = false;
            if (manualIntakePower == 0.0) {
                subsystems.intake().stop();
            } else {
                subsystems.intake().run(manualIntakePower);
            }
        }
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
