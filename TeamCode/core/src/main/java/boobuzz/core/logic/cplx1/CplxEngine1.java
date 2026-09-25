package boobuzz.core.logic.cplx1;

import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestType;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.logic.IRobotEngine;
import boobuzz.core.logic.MechanismProfile;
import boobuzz.core.subsystem.Subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

import com.pedropathing.math.Pose;

/** Complex engine: motion, automatic turret aiming, shooter sequencing, and intake. */
public final class CplxEngine1 implements IRobotEngine {

    private final Subsystems subsystems;
    private final MotionLogic motion;
    private final TurretLogic turret;
    private final ShooterLogic shooter;
    private WorldSnapshot latestWorld;
    private List<RequestStatus> pendingStatuses = Collections.emptyList();
    private RobotAction action = RobotAction.zero();
    /** Actuator owner, separate from terminal request IDs. */
    private Integer intakeOwnerId;
    /** Current manual intake demand; restored whenever a shot releases the intake. */
    private double manualIntakePower;
    private double commandedIntakePower;

    public CplxEngine1(Subsystems subsystems) {
        this(subsystems, MechanismProfile.STUB);
    }

    public CplxEngine1(Subsystems subsystems, MechanismProfile profile) {
        this.subsystems = Objects.requireNonNull(subsystems, "subsystems");
        this.turret = new TurretLogic(subsystems.turret());
        this.motion = new MotionLogic(subsystems.drive());
        this.shooter = new ShooterLogic(subsystems.shooter(), turret, profile);
    }

    @Override
    public String name() {
        return "cplx1";
    }

    @Override
    public WorldSnapshot sense(RobotState state) {
        subsystems.observe(state);
        latestWorld = new WorldSnapshot(
                state.t(), subsystems.drive().pose(), state.yaw(), state.voltage());
        subsystems.turret().setRobotPose(latestWorld.pose());
        turret.update(latestWorld.pose());
        shooter.observe(state.t(), latestWorld.pose());
        return latestWorld;
    }

    @Override
    public void act(RequestBatch batch) {
        if (batch == null) {
            batch = RequestBatch.idle();
        }
        List<RequestStatus> statuses = new ArrayList<>();
        if (containsCancelAll(batch.cancels())) {
            motion.cancelAll(statuses);
            shooter.cancelAll(statuses);
            manualIntakePower = 0.0;
            commandedIntakePower = 0.0;
            subsystems.intake().stop();
            intakeOwnerId = null;
            turret.disable();
        }
        motion.act(batch.stream(), batch.requests(), batch.cancels(), statuses);
        for (int id : batch.cancels()) {
            if (id != RequestBatch.CANCEL_ALL) {
                shooter.cancel(id, statuses);
                if (intakeOwnerId != null && intakeOwnerId == id) {
                    manualIntakePower = 0.0;
                    intakeOwnerId = null;
                }
            }
        }

        for (Request request : batch.requests()) {
            switch (request.type()) {
                case RESET_POSE -> handleResetPose(request, statuses);
                case SHOOT -> handleShoot(request, statuses);
                case SPIN_UP -> addIfRejected(statuses,
                        shooter.requestSpinUp(request.id(), request.param(0, 1.0)));
                case INTAKE, INTAKE_ON, INTAKE_OFF -> handleIntake(request, statuses);
                case TURRET_AIM -> handleTurretAim(request, statuses);
                case SET_SHOT_PRESET -> handleShotPreset(request, statuses);
                case STOP_SHOOTING -> statuses.add(shooter.stopShooting(request.id()));
                case MECHANISM_RECOVERY -> statuses.add(RequestStatus.rejected(
                        request.id(), "mechanism recovery is not implemented in cplx1 yet"));
                default -> { }
            }
        }
        shooter.update(statuses);
        applyIntake();

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

    public MotionLogic motion() {
        return motion;
    }

    public TurretLogic turret() {
        return turret;
    }

    public ShooterLogic shooter() {
        return shooter;
    }

    private void handleIntake(Request request, List<RequestStatus> statuses) {
        if (request.type() == RequestType.INTAKE_OFF
                || (request.type() == RequestType.INTAKE
                && request.param(0, 0.0) == 0.0)) {
            manualIntakePower = 0.0;
            intakeOwnerId = null;
        } else {
            double power = request.param(0, 1.0);
            manualIntakePower = Double.isFinite(power) ? power : 0.0;
            intakeOwnerId = request.id();
        }
        statuses.add(RequestStatus.done(request.id()));
    }

    /** One intake owner: an active shot commands the archive shoot power, else manual demand. */
    private void applyIntake() {
        double desired = shooter.ownsIntake() ? RobotConstants.SHOOT_INTAKE_POWER : manualIntakePower;
        if (Double.doubleToLongBits(desired) == Double.doubleToLongBits(commandedIntakePower)) {
            return;
        }
        commandedIntakePower = desired;
        if (desired == 0.0) {
            subsystems.intake().stop();
        } else {
            subsystems.intake().run(desired);
        }
    }

    private void handleTurretAim(Request request, List<RequestStatus> statuses) {
        double x = request.param(0, Double.NaN);
        double y = request.param(1, Double.NaN);
        if (request.params().length < 2 || !Double.isFinite(x) || !Double.isFinite(y)) {
            statuses.add(RequestStatus.rejected(request.id(), "TURRET_AIM requires finite field x, y"));
            return;
        }
        if (shooter.ownsTurretTarget()) {
            shooter.cancelActive("turret retargeted", statuses);
        }
        turret.setFieldTarget(x, y);
        statuses.add(RequestStatus.done(request.id()));
    }

    private void handleShotPreset(Request request, List<RequestStatus> statuses) {
        if (request.params().length < 3) {
            statuses.add(RequestStatus.rejected(request.id(),
                    "SET_SHOT_PRESET requires rpm, hoodDeg, turretRad"));
            return;
        }
        statuses.add(shooter.setPreset(request.id(), request.param(0, Double.NaN),
                request.param(1, Double.NaN), request.param(2, Double.NaN)));
    }

    private void handleShoot(Request request, List<RequestStatus> statuses) {
        if (request.params().length < 1
                || !Double.isFinite(request.param(0, Double.NaN))
                || request.param(0, 0.0) <= 0.0
                || request.param(0, 0.0) != Math.rint(request.param(0, 0.0))) {
            statuses.add(RequestStatus.rejected(
                    request.id(), "SHOOT requires a positive integer count"));
            return;
        }
        int count = (int) request.param(0, 0.0);
        RequestStatus result = request.params().length >= 2
                ? shooter.requestShot(request.id(), count, request.param(1, Double.NaN))
                : shooter.requestShot(request.id(), count);
        addIfRejected(statuses, result);
    }

    private void handleResetPose(Request request, List<RequestStatus> statuses) {
        if (request.params().length < 3
                || !Double.isFinite(request.param(0, Double.NaN))
                || !Double.isFinite(request.param(1, Double.NaN))
                || !Double.isFinite(request.param(2, Double.NaN))) {
            statuses.add(RequestStatus.rejected(
                    request.id(), "RESET_POSE requires finite x, y, heading"));
            return;
        }
        motion.resetPose(new Pose(request.param(0, 0.0), request.param(1, 0.0),
                request.param(2, 0.0)), statuses);
        shooter.localizationReset();
        statuses.add(RequestStatus.done(request.id()));
    }

    private static void addIfRejected(List<RequestStatus> statuses, RequestStatus status) {
        if (status.state() == RequestStatus.State.REJECTED
                || status.state() == RequestStatus.State.FAILED) {
            statuses.add(status);
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
}
