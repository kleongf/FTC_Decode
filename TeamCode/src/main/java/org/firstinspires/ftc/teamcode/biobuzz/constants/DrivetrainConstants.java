package org.firstinspires.ftc.teamcode.biobuzz.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DrivetrainConstants {
    public static double MOVEMENT_SPEED_MULTIPLIER = 1.0;
    public static double TURN_SPEED_MULTIPLIER = 0.4;

    public static double xP = 0.04, xI = 0, xD = 0.001;
    public static double yP = 0.04, yI = 0, yD = 0.001;
    public static double headingP = 0.5, headingI = 0, headingD = 0.035;

    public static double LOCK_TOLERANCE_POSITION = 1.0; // inches
    // Zone guide: share of the joystick in the (1-w)*PID + w*stick blend on a guided axis
    public static double GUIDE_WEIGHT_X = 0.3;
    public static double GUIDE_WEIGHT_Y = 0.3;
    public static double GUIDE_WEIGHT_HEADING = 0.3;
    // A zone only applies when its IOU with the robot footprint exceeds this
    public static double MIN_ZONE_IOU = 0.05;

    public static double LOCK_TOLERANCE_HEADING = Math.toRadians(2); // radians
}
