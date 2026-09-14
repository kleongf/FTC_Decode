package org.firstinspires.ftc.teamcode.biobuzz.constants;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.FlipUtil;

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
}
