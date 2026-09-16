package boobuzz.core.logic.direct_engine;

import boobuzz.core.contract.Drive;
import boobuzz.core.contract.Intent;
import boobuzz.core.contract.PathRequest;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestStatus;
import boobuzz.core.contract.RequestType;
import boobuzz.core.contract.RobotAction;
import boobuzz.core.contract.RobotState;
import boobuzz.core.contract.WorldSnapshot;
import boobuzz.core.logic.RobotEngine;
import boobuzz.core.subsystem.Intake;
import boobuzz.core.subsystem.Shooter;
import boobuzz.core.subsystem.Subsystems;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** Zero-intelligence engine that forwards intents directly to mechanisms. */
public final class DirectEngine implements RobotEngine {

    private final Subsystems subsystems;
    private List<RequestStatus> pendingStatuses = List.of();
    private RobotAction action = RobotAction.zero();
    private ShootJob shoot;
    private SpinJob spin;
    private GotoJob goTo;

    public DirectEngine(Subsystems subsystems) {
        this.subsystems = Objects.requireNonNull(subsystems, "subsystems");
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
    public void act(Intent intent) {
        List<RequestStatus> statuses = new ArrayList<>();
        applyDrive(intent.drive());
        for (Request request : intent.newRequests()) {
            start(request, statuses);
        }
        advance(statuses);
        pendingStatuses = List.copyOf(statuses);
        action = subsystems.update();
    }

    @Override
    public RobotAction action() {
        return action;
    }

    @Override
    public List<RequestStatus> drainStatuses() {
        List<RequestStatus> statuses = pendingStatuses;
        pendingStatuses = List.of();
        return statuses;
    }

    public Subsystems subsystems() {
        return subsystems;
    }

    private void applyDrive(Drive command) {
        if (command instanceof Drive.Manual manual) {
            subsystems.drive().manual(manual.vx(), manual.vy(), manual.omega());
        } else if (command instanceof Drive.GoTo goToCommand) {
            subsystems.drive().follow(
                    PathRequest.goTo(goToCommand.target(), goToCommand.constraints()));
        } else if (command instanceof Drive.FollowPath path) {
            subsystems.drive().follow(PathRequest.named(path.pathId()));
        } else {
            subsystems.drive().stop();
        }
    }

    private void start(Request request, List<RequestStatus> statuses) {
        RequestType type = request.type();
        switch (type) {
            case SHOOT -> startShoot(request, statuses);
            case SPIN_UP -> startSpin(request, statuses);
            case GOTO -> startGoto(request, statuses);
            case INTAKE, INTAKE_ON -> startIntake(request, statuses);
            case INTAKE_OFF -> {
                subsystems.intake().stop();
                statuses.add(RequestStatus.done(request.id()));
            }
            case PATH, TURN_TO, WAIT -> statuses.add(RequestStatus.rejected(
                    request.id(), type + " is not supported by the direct engine"));
        }
    }

    private void startShoot(Request request, List<RequestStatus> statuses) {
        if (shoot != null || spin != null) {
            statuses.add(RequestStatus.rejected(request.id(), "shooter already has a request"));
            return;
        }
        int count = (int) Math.round(request.param(0, 1.0));
        double rpm = request.param(1, 1.0);
        if (count <= 0 || !Double.isFinite(rpm) || rpm <= 0.0) {
            statuses.add(RequestStatus.rejected(request.id(), "SHOOT requires positive count and rpm"));
            return;
        }
        subsystems.shooter().spinUp(rpm);
        shoot = new ShootJob(request.id(), count, rpm);
    }

    private void startSpin(Request request, List<RequestStatus> statuses) {
        if (shoot != null || spin != null) {
            statuses.add(RequestStatus.rejected(request.id(), "shooter already has a request"));
            return;
        }
        double rpm = request.param(0, 1.0);
        if (!Double.isFinite(rpm) || rpm <= 0.0) {
            statuses.add(RequestStatus.rejected(request.id(), "SPIN_UP requires positive rpm"));
            return;
        }
        subsystems.shooter().spinUp(rpm);
        spin = new SpinJob(request.id());
    }

    private void startGoto(Request request, List<RequestStatus> statuses) {
        if (request.params() == null || request.params().length < 3) {
            statuses.add(RequestStatus.rejected(request.id(), "GOTO requires x, y, heading"));
            return;
        }
        Pose target = new Pose(request.param(0, 0), request.param(1, 0), request.param(2, 0));
        subsystems.drive().follow(PathRequest.goTo(target, Drive.Constraints.defaults()));
        goTo = new GotoJob(request.id(), false);
    }

    private void startIntake(Request request, List<RequestStatus> statuses) {
        Intake intake = subsystems.intake();
        if (request.type() == RequestType.INTAKE_ON) {
            intake.run(request.param(0, 1.0));
        } else if (request.param(0, 0.0) == 0.0) {
            intake.stop();
        } else {
            intake.run(request.param(0, 1.0));
        }
        statuses.add(RequestStatus.done(request.id()));
    }

    private void advance(List<RequestStatus> statuses) {
        Shooter shooter = subsystems.shooter();
        if (spin != null) {
            if (shooter.isReady()) {
                statuses.add(RequestStatus.done(spin.id));
                spin = null;
            } else {
                statuses.add(active(spin.id, 0.0, "spinning up"));
            }
        }
        if (shoot != null) {
            shooter.spinUp(shoot.rpm);
            if (shooter.isFeeding()) {
                statuses.add(active(shoot.id, 0.5, "feeding"));
            } else if (!shooter.isReady()) {
                statuses.add(active(shoot.id, 0.0, "spinning up"));
            } else if (shoot.remaining > 0) {
                shooter.feed();
                if (shooter.isFeeding()) {
                    shoot.remaining--;
                    statuses.add(active(shoot.id, 0.5, "feeding"));
                } else {
                    statuses.add(active(shoot.id, 0.0, "feed not accepted"));
                }
            } else {
                statuses.add(RequestStatus.done(shoot.id));
                shoot = null;
            }
        }
        if (goTo != null) {
            if (!goTo.started) {
                goTo.started = true;
                statuses.add(active(goTo.id, 0.0, "following"));
            } else if (subsystems.drive().pathDone()) {
                statuses.add(RequestStatus.done(goTo.id));
                goTo = null;
            } else {
                statuses.add(active(goTo.id, 0.0, "following"));
            }
        }
    }

    private static RequestStatus active(int id, double progress, String note) {
        return new RequestStatus(id, RequestStatus.State.ACTIVE, progress, note);
    }

    private static final class ShootJob {
        private final int id;
        private final double rpm;
        private int remaining;

        private ShootJob(int id, int remaining, double rpm) {
            this.id = id;
            this.remaining = remaining;
            this.rpm = rpm;
        }
    }

    private static final class SpinJob {
        private final int id;

        private SpinJob(int id) {
            this.id = id;
        }
    }

    private static final class GotoJob {
        private final int id;
        private boolean started;

        private GotoJob(int id, boolean started) {
            this.id = id;
            this.started = started;
        }
    }
}
