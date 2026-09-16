package boobuzz.core.controller.opmodes;

import boobuzz.core.controller.auto.AutoBuilder;
import boobuzz.core.controller.auto.AutoSequence;

/** Blue Missionary nine-piece routine; the archive's low lane remains disabled. */
public final class BlueMissionary9Piece {

    private static final double SETTLE_SECONDS = 0.2;
    private static final double INTAKE_EXTRA_SECONDS = 0.3;

    private BlueMissionary9Piece() {}

    public static AutoSequence build() {
        return AutoBuilder.start(AutoLocations.Blue.START_MISSIONARY)
                .goToPose(AutoLocations.Blue.SCORE_BIG_HIGH)
                .waitSeconds(SETTLE_SECONDS)
                .shoot(3)
                .goToPose(AutoLocations.Blue.PICKUP_HIGH_START)
                .goToPose(AutoLocations.Blue.PICKUP_HIGH_END)
                .withIntake().withBraking(0.85, 0.9)
                .intake(INTAKE_EXTRA_SECONDS)
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
                .goToPose(AutoLocations.Blue.BIG_END)
                .build();
    }
}
