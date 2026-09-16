package boobuzz.core.controller.autos;

import com.pedropathing.math.Pose;

/** Field coordinates ported from last season's lvbelc5 autonomous routines. */
public final class AutoLocations {

    private static final double FIELD_WIDTH = 144.0;
    private static final double ROBOT_WIDTH = 400.0 / 25.4;
    private static final double ROBOT_FRONT = 247.0 / 25.4;
    private static final double ROBOT_BACK = 202.0 / 25.4;
    private static final double HALF_WIDTH = ROBOT_WIDTH / 2.0;

    private AutoLocations() {}

    public static final double LEVER_X_OFFSET = 0.0;
    public static final double LEVER_Y_OFFSET = 3.0;
    public static final double LEVER_WAIT_SECONDS = 1.0;
    public static final double BALL_RADIUS = 2.5;
    public static final double INTAKE_START_OFFSET = 18.0;
    public static final double INTAKE_END_OFFSET = 0.5;

    public static final class Balls {
        private Balls() {}

        public static final double BALL_SPACING = 3.0 * BALL_RADIUS;
        public static final double HIGH_Y = 84.0;
        public static final double MID_Y = 60.0;
        public static final double LOW_Y = 36.0;
        public static final double HUMAN_OFFSET = 1.5;
        public static final double HUMAN_Y = ROBOT_WIDTH / 2.0 + HUMAN_OFFSET;
        public static final double BLUE_CENTER_X = 24.0;
        public static final double BLUE_INNER_X = BLUE_CENTER_X + BALL_SPACING;
        public static final double BLUE_OUTER_X = BLUE_CENTER_X - BALL_SPACING;

        public static final Pose BLUE_HIGH_OUTER = pose(BLUE_OUTER_X, HIGH_Y, 0.0);
        public static final Pose BLUE_HIGH_CENTER = pose(BLUE_CENTER_X, HIGH_Y, 0.0);
        public static final Pose BLUE_HIGH_INNER = pose(BLUE_INNER_X, HIGH_Y, 0.0);
        public static final Pose BLUE_MID_OUTER = pose(BLUE_OUTER_X, MID_Y, 0.0);
        public static final Pose BLUE_MID_CENTER = pose(BLUE_CENTER_X, MID_Y, 0.0);
        public static final Pose BLUE_MID_INNER = pose(BLUE_INNER_X, MID_Y, 0.0);
        public static final Pose BLUE_LOW_OUTER = pose(BLUE_OUTER_X, LOW_Y, 0.0);
        public static final Pose BLUE_LOW_CENTER = pose(BLUE_CENTER_X, LOW_Y, 0.0);
        public static final Pose BLUE_LOW_INNER = pose(BLUE_INNER_X, LOW_Y, 0.0);
        public static final Pose BLUE_HUMAN_CENTER = pose(BLUE_CENTER_X, HUMAN_Y, 0.0);

        public static final Pose RED_HIGH_OUTER = mirror(BLUE_HIGH_OUTER);
        public static final Pose RED_HIGH_CENTER = mirror(BLUE_HIGH_CENTER);
        public static final Pose RED_HIGH_INNER = mirror(BLUE_HIGH_INNER);
        public static final Pose RED_MID_OUTER = mirror(BLUE_MID_OUTER);
        public static final Pose RED_MID_CENTER = mirror(BLUE_MID_CENTER);
        public static final Pose RED_MID_INNER = mirror(BLUE_MID_INNER);
        public static final Pose RED_LOW_OUTER = mirror(BLUE_LOW_OUTER);
        public static final Pose RED_LOW_CENTER = mirror(BLUE_LOW_CENTER);
        public static final Pose RED_LOW_INNER = mirror(BLUE_LOW_INNER);
        public static final Pose RED_HUMAN_CENTER = mirror(BLUE_HUMAN_CENTER);
    }

    public static final class Blue {
        private Blue() {}

        public static final Pose START_MISSIONARY = poseFromFrontLeft(24.0, 144.0, 90.0);
        public static final Pose START_DOGGY = poseFromBackLeft(48.0, 0.0, 90.0);
        public static final Pose SCORE_BIG_HIGH = pose(48.0, 96.0, 90.0);
        public static final Pose SCORE_BIG_MID = pose(48.0, 96.0, 90.0);
        public static final Pose SCORE_BIG_LOW = pose(48.0, 96.0, 90.0);
        public static final Pose SCORE_BIG_PICKUP = pose(48.0, 96.0, 90.0);
        public static final Pose SCORE_LITTLE_LOW = pose(60.0, 12.0, 90.0);
        public static final Pose SCORE_LITTLE_PRELOAD = SCORE_LITTLE_LOW;
        public static final Pose SCORE_LITTLE_HUMAN = pose(60.0, 12.0, 180.0);

        public static final Pose PICKUP_HIGH_START = pose(
                Balls.BLUE_INNER_X + INTAKE_START_OFFSET, Balls.HIGH_Y, 180.0);
        public static final Pose PICKUP_HIGH_END = pose(
                Balls.BLUE_OUTER_X - INTAKE_END_OFFSET, Balls.HIGH_Y, 180.0);
        public static final Pose PICKUP_MID_START = pose(
                Balls.BLUE_INNER_X + INTAKE_START_OFFSET, Balls.MID_Y, 180.0);
        public static final Pose PICKUP_MID_END = pose(
                Balls.BLUE_OUTER_X - INTAKE_END_OFFSET, Balls.MID_Y, 180.0);
        public static final Pose PICKUP_LOW_START = pose(
                Balls.BLUE_INNER_X + INTAKE_START_OFFSET, Balls.LOW_Y, 180.0);
        public static final Pose PICKUP_LOW_END = pose(
                Balls.BLUE_OUTER_X - INTAKE_END_OFFSET, Balls.LOW_Y, 180.0);
        public static final Pose PICKUP_HUMAN_START = pose(
                Balls.BLUE_INNER_X + INTAKE_START_OFFSET, Balls.HUMAN_Y, 180.0);
        public static final Pose PICKUP_HUMAN_END = pose(
                Balls.BLUE_OUTER_X - INTAKE_END_OFFSET, Balls.HUMAN_Y, 180.0);

        public static final Pose BIG_END = poseFromBackLeft(48.0, 108.0, 90.0);
        public static final Pose BIG_END_LEVER = poseFromBackLeft(45.0, 72.0, 90.0);
        public static final Pose LITTLE_END = poseFromBackRight(48.0, 24.0, 180.0);
        public static final Pose LEVER = poseFromFrontLeft(
                10.0 + LEVER_X_OFFSET, 82.0 + LEVER_Y_OFFSET, 90.0);
        public static final Pose PARK = pose(105.0, 33.0, 90.0);
    }

    public static final class Red {
        private Red() {}

        public static final Pose START_MISSIONARY = mirror(Blue.START_MISSIONARY);
        public static final Pose START_DOGGY = mirror(Blue.START_DOGGY);
        public static final Pose SCORE_BIG_HIGH = pose(96.0, 96.0, 90.0);
        public static final Pose SCORE_BIG_MID = pose(96.0, 96.0, 90.0);
        public static final Pose SCORE_BIG_LOW = pose(96.0, 96.0, 90.0);
        public static final Pose SCORE_BIG_PICKUP = pose(96.0, 96.0, 90.0);
        public static final Pose SCORE_LITTLE_LOW = pose(84.0, 12.0, 90.0);
        public static final Pose SCORE_LITTLE_PRELOAD = SCORE_LITTLE_LOW;
        public static final Pose SCORE_LITTLE_HUMAN = pose(84.0, 12.0, 0.0);

        public static final Pose PICKUP_HIGH_START = mirror(Blue.PICKUP_HIGH_START);
        public static final Pose PICKUP_HIGH_END = mirror(Blue.PICKUP_HIGH_END);
        public static final Pose PICKUP_MID_START = mirror(Blue.PICKUP_MID_START);
        public static final Pose PICKUP_MID_END = mirror(Blue.PICKUP_MID_END);
        public static final Pose PICKUP_LOW_START = mirror(Blue.PICKUP_LOW_START);
        public static final Pose PICKUP_LOW_END = mirror(Blue.PICKUP_LOW_END);
        public static final Pose PICKUP_HUMAN_START = mirror(Blue.PICKUP_HUMAN_START);
        public static final Pose PICKUP_HUMAN_END = mirror(Blue.PICKUP_HUMAN_END);
        public static final Pose BIG_END = mirror(Blue.BIG_END);
        public static final Pose BIG_END_LEVER = mirror(Blue.BIG_END_LEVER);
        public static final Pose LITTLE_END = mirror(Blue.LITTLE_END);
        public static final Pose LEVER = mirror(Blue.LEVER);
        public static final Pose PARK = mirror(Blue.PARK);
    }

    public static Pose pose(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    public static Pose mirror(Pose value) {
        return new Pose(FIELD_WIDTH - value.x(), value.y(), Math.PI - value.heading());
    }

    public static Pose poseFromFrontLeft(double cornerX, double cornerY, double headingDeg) {
        return fromCorner(cornerX, cornerY, headingDeg, -ROBOT_FRONT, HALF_WIDTH);
    }

    public static Pose poseFromFrontRight(double cornerX, double cornerY, double headingDeg) {
        return fromCorner(cornerX, cornerY, headingDeg, -ROBOT_FRONT, -HALF_WIDTH);
    }

    public static Pose poseFromBackLeft(double cornerX, double cornerY, double headingDeg) {
        return fromCorner(cornerX, cornerY, headingDeg, ROBOT_BACK, HALF_WIDTH);
    }

    public static Pose poseFromBackRight(double cornerX, double cornerY, double headingDeg) {
        return fromCorner(cornerX, cornerY, headingDeg, ROBOT_BACK, -HALF_WIDTH);
    }

    private static Pose fromCorner(double cornerX, double cornerY, double headingDeg,
                                   double forwardOffset, double leftOffset) {
        double heading = Math.toRadians(headingDeg);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        return new Pose(cornerX + forwardOffset * cos + leftOffset * sin,
                cornerY + forwardOffset * sin - leftOffset * cos, heading);
    }
}
