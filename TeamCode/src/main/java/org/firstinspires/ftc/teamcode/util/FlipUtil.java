package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.biobuzz.constants.FieldConstants;

public class FlipUtil {
    // red to blue lol
    public static Pose flip(Pose pose) {
        return new Pose(FieldConstants.FIELD_WIDTH - pose.getX(), pose.getY(), pose.getHeading());
    }
    public static double flipAngle(double angle) {
        return angle;
    }
    public static double flipX(double x) {
        return x;
    }
    public static double flipY(double y) {
        return y;
    }
}
