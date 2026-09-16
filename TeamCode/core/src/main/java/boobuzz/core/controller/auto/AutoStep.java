package boobuzz.core.controller.auto;

import boobuzz.core.contract.PathRequest;

import com.pedropathing.math.Pose;

import java.util.Objects;

/** One immutable operation in an autonomous sequence. */
public sealed interface AutoStep
        permits AutoStep.Path, AutoStep.Turn, AutoStep.Wait,
                AutoStep.Shoot, AutoStep.Intake, AutoStep.SpinUp {

    String name();

    /** A path plus modifiers attached by the fluent builder. */
    record Path(PathRequest request, Pose startPose,
                boolean intake, double intakePower, Double shooterWarmupRpm)
            implements AutoStep {
        public Path {
            Objects.requireNonNull(request, "path request");
            Objects.requireNonNull(startPose, "path start pose");
            if (intake && (!Double.isFinite(intakePower) || intakePower == 0.0)) {
                throw new IllegalArgumentException("intake power must be finite and non-zero");
            }
            if (shooterWarmupRpm != null
                    && (!Double.isFinite(shooterWarmupRpm) || shooterWarmupRpm <= 0.0)) {
                throw new IllegalArgumentException("shooter warmup RPM must be positive and finite");
            }
        }

        public Path(PathRequest request) {
            this(request, Pose.zero(), false, 0.8, null);
        }

        public Path(PathRequest request, double startHeadingRad,
                    boolean intake, double intakePower, Double shooterWarmupRpm) {
            this(request, new Pose(0.0, 0.0, startHeadingRad), intake, intakePower,
                    shooterWarmupRpm);
        }

        public double startHeadingRad() {
            return startPose.heading();
        }

        @Override
        public String name() {
            return "Path";
        }
    }

    record Turn(double headingRad) implements AutoStep {
        public Turn {
            if (!Double.isFinite(headingRad)) {
                throw new IllegalArgumentException("turn heading must be finite");
            }
        }

        @Override
        public String name() {
            return String.format(java.util.Locale.US, "Turn(%.1f°)",
                    Math.toDegrees(headingRad));
        }
    }

    record Wait(double seconds) implements AutoStep {
        public Wait {
            if (!Double.isFinite(seconds) || seconds < 0.0) {
                throw new IllegalArgumentException("wait seconds must be finite and non-negative");
            }
        }

        @Override
        public String name() {
            return String.format(java.util.Locale.US, "Wait(%.1fs)", seconds);
        }
    }

    record Shoot(int count, double rpm) implements AutoStep {
        public Shoot {
            if (count <= 0) {
                throw new IllegalArgumentException("shoot count must be positive");
            }
            if (!Double.isFinite(rpm) || rpm <= 0.0) {
                throw new IllegalArgumentException("shoot RPM must be positive and finite");
            }
        }

        public Shoot(int count) {
            this(count, 1.0);
        }

        @Override
        public String name() {
            return "Shoot(" + count + ")";
        }
    }

    record Intake(double seconds, double power) implements AutoStep {
        public Intake {
            if (!Double.isFinite(seconds) || seconds < 0.0) {
                throw new IllegalArgumentException("intake seconds must be finite and non-negative");
            }
            if (!Double.isFinite(power) || power == 0.0) {
                throw new IllegalArgumentException("intake power must be finite and non-zero");
            }
        }

        public Intake(double seconds) {
            this(seconds, 0.8);
        }

        @Override
        public String name() {
            return String.format(java.util.Locale.US, "Intake(%.1fs)", seconds);
        }
    }

    record SpinUp(double rpm) implements AutoStep {
        public SpinUp {
            if (!Double.isFinite(rpm) || rpm <= 0.0) {
                throw new IllegalArgumentException("spin-up RPM must be positive and finite");
            }
        }

        @Override
        public String name() {
            return String.format(java.util.Locale.US, "SpinUp(%.0f)", rpm);
        }
    }
}
