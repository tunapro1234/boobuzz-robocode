package boobuzz.core.controller.autos;

import boobuzz.core.controller.auto.AutoBuilder;
import boobuzz.core.controller.auto.AutoSequence;

import com.pedropathing.math.Pose;

/** Blue Missionary lever routine ported from last season's data. */
public final class BlueMissionary9PieceLever {

    private static final double SETTLE_SECONDS = 0.2;
    private static final double INTAKE_EXTRA_SECONDS = 0.3;

    private BlueMissionary9PieceLever() {}

    public static AutoSequence build() {
        return AutoBuilder.start(AutoLocations.Blue.START_MISSIONARY)
                .goToPose(AutoLocations.Blue.SCORE_BIG_HIGH)
                .waitSeconds(SETTLE_SECONDS)
                .shoot(3)
                .goToPose(AutoLocations.Blue.PICKUP_HIGH_START)
                .goToPose(AutoLocations.Blue.PICKUP_HIGH_END)
                .withIntake().withBraking(0.85, 0.9)
                .intake(INTAKE_EXTRA_SECONDS)
                .goToPose(new Pose(34.0, 84.0, Math.PI))
                .goToPose(AutoLocations.Blue.LEVER)
                .waitSeconds(AutoLocations.LEVER_WAIT_SECONDS)
                .goToPose(AutoLocations.Blue.SCORE_BIG_HIGH)
                .withShooterWarmup()
                .waitSeconds(SETTLE_SECONDS)
                .shoot(3)
                .goToPose(AutoLocations.Blue.PICKUP_MID_START)
                .goToPose(AutoLocations.Blue.PICKUP_MID_END)
                .withIntake().withBraking(0.85, 0.9)
                .intake(INTAKE_EXTRA_SECONDS)
                .goToPose(AutoLocations.Blue.SCORE_BIG_MID)
                .withShooterWarmup()
                .waitSeconds(SETTLE_SECONDS)
                .shoot(3)
                // Low lane remains disabled in the source routine.
                // .goToPose(AutoLocations.Blue.PICKUP_LOW_START)
                // .goToPose(AutoLocations.Blue.PICKUP_LOW_END).withIntake()
                // .intake(INTAKE_EXTRA_SECONDS)
                // .goToPose(AutoLocations.Blue.SCORE_BIG_LOW).withShooterWarmup()
                // .waitSeconds(SETTLE_SECONDS)
                // .shoot(3)
                .goToPose(AutoLocations.Blue.BIG_END_LEVER)
                .build();
    }
}
