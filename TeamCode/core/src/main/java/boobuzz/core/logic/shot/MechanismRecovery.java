package boobuzz.core.logic.shot;

import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.hal.RobotConstants;
import boobuzz.core.subsystem.IShooter;

import java.util.List;
import java.util.Objects;

/**
 * MECHANISM_RECOVERY owner shared by both engines (phase-1.1-a request addendum). Every
 * mode is operator-held: nothing here starts, repeats or retries on its own.
 *
 * <ul>
 *   <li>Mode 1 (archive LB, ShootingController.executeJamClear): intake and feeder at -1;
 *       the flywheel is left alone. An active shot stays active but feeds nothing.</li>
 *   <li>Mode 2 (archive Y, executeShooterJamClear): flywheel open loop and feeder at +-1,
 *       intake +1, hood 25 deg, sign toggled every 500 ms. The engine cancels any shooter
 *       request first. Spec B08 safety correction: every sign change waits until the
 *       flywheel is measured stopped (the archive toggled instantly).</li>
 *   <li>Mode 0: exit. Leaving a jam clear coasts the flywheel; that request finishes only
 *       once the flywheel is stopped, and shooter requests are refused until then.</li>
 * </ul>
 * A new mode 1/2 request supersedes the active one (DONE "superseded").
 */
public final class MechanismRecovery {

    public enum Phase { OFF, REVERSE, JAM_RUN, JAM_SPINDOWN, EXIT_SPINDOWN }

    private final IShooter shooter;
    private Phase phase = Phase.OFF;
    /** Request that receives the terminal status; -1 once it already has one. */
    private int requestId = -1;
    private long nowMs;
    private long phaseStartMs;
    private double jamSign = 1.0;

    public MechanismRecovery(IShooter shooter) {
        this.shooter = Objects.requireNonNull(shooter, "shooter");
    }

    public void observe(long tMs) {
        nowMs = tMs;
    }

    /** Parsed mode 0, 1 or 2, or -1 when the request is malformed. */
    public static int mode(Request request) {
        double mode = request.param(0, Double.NaN);
        if (request.params().length < 1 || !Double.isFinite(mode) || mode != Math.rint(mode)
                || mode < 0.0 || mode > 2.0) {
            return -1;
        }
        return (int) mode;
    }

    /**
     * Handles one MECHANISM_RECOVERY request after {@link #mode} validated it. For mode 2
     * the caller must already have cancelled any active shooter request.
     */
    public void request(int id, int mode, List<RequestStatus> statuses) {
        if (mode == 0) {
            exit(statuses);
            statuses.add(RequestStatus.done(id));
            return;
        }
        if (requestId >= 0) {
            statuses.add(new RequestStatus(requestId, RequestStatus.State.DONE, 1.0, "superseded"));
        }
        requestId = id;
        if (mode == 1) {
            if (jamPhase()) {
                // Leaving a jam clear: coast the flywheel; any later spin-up is guarded
                // against the remaining reverse rotation by the shooter itself.
                shooter.spinDown();
            }
            enter(Phase.REVERSE);
        } else {
            jamSign = 1.0;
            enter(Phase.JAM_RUN);
        }
    }

    /** Per-request cancel; a jam clear still coasts to a stop before shooting is allowed. */
    public boolean cancel(int id, List<RequestStatus> statuses) {
        if (requestId < 0 || requestId != id) {
            return false;
        }
        statuses.add(RequestStatus.rejected(id, "cancelled"));
        requestId = -1;
        stopOutputs();
        return true;
    }

    /** CANCEL_ALL: immediate stop; a spinning jam-clear wheel still blocks shooting. */
    public void cancelAll(List<RequestStatus> statuses) {
        if (requestId >= 0) {
            statuses.add(RequestStatus.rejected(requestId, "engine switch"));
        }
        requestId = -1;
        stopOutputs();
    }

    /** Commands this tick's recovery outputs; call after the shot coordinator. */
    public void update(List<RequestStatus> statuses) {
        switch (phase) {
            case OFF -> { }
            case REVERSE -> {
                shooter.setFeederPower(RobotConstants.RECOVERY_REVERSE_POWER);
                active(statuses, "reversing");
            }
            case JAM_RUN -> {
                if (nowMs - phaseStartMs >= RobotConstants.JAM_CLEAR_TOGGLE_MS) {
                    enter(Phase.JAM_SPINDOWN);
                    updateSpinDown(statuses);
                } else {
                    runJam(statuses);
                }
            }
            case JAM_SPINDOWN -> updateSpinDown(statuses);
            case EXIT_SPINDOWN -> {
                shooter.spinDown();
                if (shooter.isStopped()) {
                    if (requestId >= 0) {
                        statuses.add(new RequestStatus(requestId, RequestStatus.State.DONE,
                                1.0, "exited"));
                    }
                    requestId = -1;
                    phase = Phase.OFF;
                } else {
                    active(statuses, "flywheel coasting");
                }
            }
        }
    }

    public Phase phase() {
        return phase;
    }

    public boolean active() {
        return phase != Phase.OFF;
    }

    /** An active shot must not start a pulse while any recovery owns the feeder. */
    public boolean holdsFeed() {
        return phase != Phase.OFF;
    }

    /** New SHOOT/SPIN_UP requests are refused while the flywheel belongs to recovery. */
    public boolean blocksShooter() {
        return jamPhase() || phase == Phase.EXIT_SPINDOWN;
    }

    /** True while recovery owns the intake output; see {@link #intakePower()}. */
    public boolean ownsIntake() {
        return phase == Phase.REVERSE || jamPhase();
    }

    public double intakePower() {
        if (phase == Phase.REVERSE) {
            return RobotConstants.RECOVERY_REVERSE_POWER;
        }
        return jamPhase() ? RobotConstants.JAM_CLEAR_POWER : 0.0;
    }

    private void exit(List<RequestStatus> statuses) {
        if (phase == Phase.REVERSE) {
            shooter.setFeederPower(0.0);
            if (requestId >= 0) {
                statuses.add(new RequestStatus(requestId, RequestStatus.State.DONE, 1.0, "exited"));
            }
            requestId = -1;
            phase = Phase.OFF;
        } else if (jamPhase()) {
            enter(Phase.EXIT_SPINDOWN);
            shooter.spinDown();
        }
    }

    private void stopOutputs() {
        if (phase == Phase.REVERSE) {
            shooter.setFeederPower(0.0);
            phase = Phase.OFF;
        } else if (phase != Phase.OFF) {
            shooter.spinDown();
            phase = Phase.EXIT_SPINDOWN;
        }
    }

    private void updateSpinDown(List<RequestStatus> statuses) {
        shooter.runOpenLoop(0.0);
        shooter.setFeederPower(0.0);
        shooter.setHoodAngleDeg(RobotConstants.JAM_CLEAR_HOOD_DEG);
        if (shooter.isStopped()) {
            jamSign = -jamSign;
            enter(Phase.JAM_RUN);
            runJam(statuses);
        } else {
            active(statuses, "jam clear: waiting for stop");
        }
    }

    private void runJam(List<RequestStatus> statuses) {
        shooter.runOpenLoop(jamSign * RobotConstants.JAM_CLEAR_POWER);
        shooter.setFeederPower(jamSign * RobotConstants.JAM_CLEAR_POWER);
        shooter.setHoodAngleDeg(RobotConstants.JAM_CLEAR_HOOD_DEG);
        active(statuses, jamSign > 0.0 ? "jam clear forward" : "jam clear reverse");
    }

    private void enter(Phase next) {
        phase = next;
        phaseStartMs = nowMs;
    }

    private boolean jamPhase() {
        return phase == Phase.JAM_RUN || phase == Phase.JAM_SPINDOWN;
    }

    private void active(List<RequestStatus> statuses, String note) {
        if (requestId >= 0) {
            statuses.add(new RequestStatus(requestId, RequestStatus.State.ACTIVE, 0.0, note));
        }
    }
}
