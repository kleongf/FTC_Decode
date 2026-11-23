package org.firstinspires.ftc.teamcode.biobuzz.constants;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.lib.zones.HeadingTarget;
import org.firstinspires.ftc.teamcode.lib.zones.Zone;
import org.firstinspires.ftc.teamcode.util.FlipUtil;

import java.util.Arrays;
import java.util.List;

public class FieldConstants {
    public static String END_POSE_KEY = "END_POSE";
    public static double ROBOT_WIDTH = 18d;
    public static double ROBOT_LENGTH = 18d;
    public static double FIELD_WIDTH = 144d;
    public static double DISTANCE_TO_CENTER = 15d;

    // TODO: set to the real alliance-specific starting poses once re-tuned for this season
    public static Pose BLUE_STANDARD_START_POSE = new Pose(0, 0, 0);
    public static Pose RED_STANDARD_START_POSE = new Pose(0, 0, 0);
    public static Pose RED_SHOOT_POSE = new Pose(FIELD_WIDTH * (5/12d), FIELD_WIDTH / 2 - DISTANCE_TO_CENTER);
    public static Pose BLUE_SHOOT_POSE = FlipUtil.flip(RED_SHOOT_POSE);

    public static Pose RED_CLOSE_START_POSE = new Pose(FIELD_WIDTH * (2/6d) + ROBOT_WIDTH / 2, ROBOT_LENGTH / 2, Math.toRadians(90));
    public static Pose RED_FAR_START_POSE = new Pose(FIELD_WIDTH * (2/6d) + ROBOT_WIDTH / 2, FIELD_WIDTH - ROBOT_LENGTH / 2, Math.toRadians(-90));

    // Guide zones: a pair straddles each of the lines x = 1/3 and x = 2/3 of the field. The zone on
    // the left of a line pushes the robot left of it, the one on the right pushes it right.
    // Same for both alliances (symmetric about the field center).
    public static double ZONE_WIDTH = 10d;
    public static double ZONE_HEIGHT = 60d;
    public static double ZONE_PUSH_MARGIN = 0d;
    public static List<Zone> PUSH_ZONES = Arrays.asList(
            pushLeftOf(FIELD_WIDTH / 3), pushRightOf(FIELD_WIDTH / 3),
            pushLeftOf(FIELD_WIDTH * 2 / 3), pushRightOf(FIELD_WIDTH * 2 / 3)
    );

    private static Zone pushLeftOf(double lineX) {
        return Zone.pushX(lineX - ZONE_WIDTH / 2, FIELD_WIDTH / 2, ZONE_WIDTH, ZONE_HEIGHT,
                lineX - ROBOT_WIDTH / 2 - ZONE_PUSH_MARGIN, HeadingTarget.nearest90());
    }

    private static Zone pushRightOf(double lineX) {
        return Zone.pushX(lineX + ZONE_WIDTH / 2, FIELD_WIDTH / 2, ZONE_WIDTH, ZONE_HEIGHT,
                lineX + ROBOT_WIDTH / 2 + ZONE_PUSH_MARGIN, HeadingTarget.nearest90());
    }
}
