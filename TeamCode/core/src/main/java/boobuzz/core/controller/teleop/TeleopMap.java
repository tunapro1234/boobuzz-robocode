package boobuzz.core.controller.teleop;

import boobuzz.core.controller.Buttons;
import boobuzz.core.controller.auto.AutoBuilder;
import boobuzz.core.controller.auto.AutoSequence;
import boobuzz.core.contract.Feedback;
import boobuzz.core.contract.GamepadState;
import boobuzz.core.contract.Request;
import boobuzz.core.contract.RequestBatch;
import boobuzz.core.contract.RequestStream;
import boobuzz.core.hal.RobotConstants;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.List;

/**
 * Editable Phase 1.1 gamepad map.  Each control is kept beside the request it
 * emits so changes to the driver contract stay visible in one small file.
 */
public final class TeleopMap {

    private static final double DEADBAND = 0.05;
    private static final int FIRST_REQUEST_ID = 10_000;

    /** The mapped tick plus an optional fluent sequence started by Y. */
    public record Mapping(RequestBatch batch, AutoSequence sequenceToStart) {
        public Mapping {
            batch = batch == null ? RequestBatch.idle() : batch;
        }
    }

    private boolean fieldOriented = true;
    private double headingOffset;
    private boolean intakeOn;
    private boolean engineSwitchSent;
    private int nextEngineIndex = 1;
    private int nextRequestId = FIRST_REQUEST_ID;

    /**
     * Maps one pair of gamepad frames.  {@code feedback} supplies the current
     * heading for field-oriented driving and the start pose for the Y sequence.
     */
    public Mapping map(Buttons buttons, Feedback feedback) {
        GamepadState g = buttons.current();
        List<Request> requests = new ArrayList<>(3);

        // B remains the field/robot-oriented toggle from the original TeleOp.
        if (buttons.pressed("b")) {
            fieldOriented = !fieldOriented;
        }

        // Preserve the original heading-reset behavior while Y also starts the
        // fluent sequence below.
        if (buttons.pressed("y")) {
            headingOffset = rawHeading(feedback);
        }

        // Left stick -> stream drive; right-stick X -> stream omega.
        double forward = deadband(-g.ly());
        double left = deadband(-g.lx());
        double omega = deadband(-g.rx());
        double vx = forward;
        double vy = left;
        double heading = rawHeading(feedback) - headingOffset;
        if (fieldOriented) {
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            vx = forward * cos + left * sin;
            vy = -forward * sin + left * cos;
        }
        boolean manualInput = Math.abs(g.lx()) > 0.0
                || Math.abs(g.ly()) > 0.0
                || Math.abs(g.rx()) > 0.0;
        RequestStream stream = new RequestStream(vx, vy, omega, manualInput);

        // Right bumper pressed -> one three-piece shot request.
        if (buttons.pressed("rb")) {
            requests.add(Request.shoot(allocateId(), 3));
        }

        // Left bumper toggle -> explicit intake ON/OFF requests.
        if (buttons.pressed("lb")) {
            intakeOn = buttons.toggle("lb");
            requests.add(intakeOn
                    ? Request.intakeOn(allocateId(), 1.0)
                    : Request.intakeOff(allocateId()));
        }

        // Y resets the heading offset and starts a short fluent auto sequence.
        AutoSequence sequence = null;
        if (buttons.pressed("y")) {
            sequence = buildYSequence(feedback);
        }

        // BACK -> fixed compile-time pose reset.
        if (buttons.pressed("back")) {
            requests.add(Request.resetPose(allocateId(),
                    RobotConstants.TELEOP_RESET_POSE_X,
                    RobotConstants.TELEOP_RESET_POSE_Y,
                    RobotConstants.TELEOP_RESET_POSE_H));
        }

        // START held for one second -> one engine-switch request.
        if (!buttons.held("start")) {
            engineSwitchSent = false;
        } else if (!engineSwitchSent && buttons.heldFor("start", 1.0)) {
            engineSwitchSent = true;
            nextEngineIndex = nextEngineIndex == 0 ? 1 : 0;
            requests.add(Request.switchEngine(allocateId(), nextEngineIndex));
        }

        return new Mapping(new RequestBatch(stream, requests), sequence);
    }

    /** Convenience form for tests and tools that already have two frames. */
    public Mapping map(GamepadState previous, GamepadState current,
                       long previousTimeMs, long currentTimeMs, Feedback feedback) {
        return map(new Buttons(previous, current, previousTimeMs, currentTimeMs), feedback);
    }

    /** Returns only the mapped batch, discarding a Y-started sequence. */
    public RequestBatch batch(Buttons buttons, Feedback feedback) {
        return map(buttons, feedback).batch();
    }

    private AutoSequence buildYSequence(Feedback feedback) {
        Pose pose = feedback == null || feedback.world() == null
                || feedback.world().pose() == null
                ? new Pose(RobotConstants.TELEOP_RESET_POSE_X,
                RobotConstants.TELEOP_RESET_POSE_Y,
                RobotConstants.TELEOP_RESET_POSE_H)
                : feedback.world().pose();
        double headingDeg = Math.toDegrees(pose.heading());
        return AutoBuilder.start(pose)
                .goTo(pose.x() + 12.0, pose.y(), headingDeg)
                .withConstantHeading(headingDeg)
                .shoot(3)
                .build();
    }

    private int allocateId() {
        return nextRequestId++;
    }

    private static double rawHeading(Feedback feedback) {
        if (feedback == null || feedback.world() == null) {
            return 0.0;
        }
        Pose pose = feedback.world().pose();
        return pose == null ? feedback.world().yaw() : pose.heading();
    }

    private static double deadband(double value) {
        if (Math.abs(value) < DEADBAND) {
            return 0.0;
        }
        double sign = Math.signum(value);
        return sign * (Math.abs(value) - DEADBAND) / (1.0 - DEADBAND);
    }
}
