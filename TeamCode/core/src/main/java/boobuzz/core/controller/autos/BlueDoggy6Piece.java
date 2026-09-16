package boobuzz.core.controller.autos;

import boobuzz.core.controller.auto.AutoBuilder;
import boobuzz.core.controller.auto.AutoSequence;

/** Blue Doggy six-piece routine ported from last season's data. */
public final class BlueDoggy6Piece {

    private static final double SETTLE_SECONDS = 0.2;
    private static final double PRELOAD_SETTLE_SECONDS = 0.6;
    private static final double INTAKE_EXTRA_SECONDS = 0.3;

    private BlueDoggy6Piece() {}

    public static AutoSequence build() {
        return AutoBuilder.start(AutoLocations.Blue.START_DOGGY)
                .goToPose(AutoLocations.Blue.SCORE_LITTLE_PRELOAD)
                .waitSeconds(PRELOAD_SETTLE_SECONDS)
                .shoot(3)
                .goToPose(AutoLocations.Blue.PICKUP_LOW_START)
                .goToPose(AutoLocations.Blue.PICKUP_LOW_END)
                .withIntake().withBraking(0.85, 0.9)
                .intake(INTAKE_EXTRA_SECONDS)
                .goToPose(AutoLocations.Blue.SCORE_LITTLE_LOW)
                .withShooterWarmup()
                .waitSeconds(SETTLE_SECONDS)
                .shoot(3)
                .goToPose(AutoLocations.Blue.LITTLE_END)
                .build();
    }
}
