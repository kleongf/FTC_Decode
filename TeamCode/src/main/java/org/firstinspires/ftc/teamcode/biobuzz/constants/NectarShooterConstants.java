package org.firstinspires.ftc.teamcode.biobuzz.constants;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class NectarShooterConstants {
    public static String SHOOTER_MOTOR_NAME = "POLLEN_SHOOTER_MOTOR";
    public static String PITCH_SERVO_NAME = "POLLEN_PITCH_SERVO";
    public static String LATCH_SERVO_NAME = "POLLEN_LATCH_SERVO";
    public static String TURRET_SERVO_NAME = "POLLEN_TURRET_SERVO";
    public static double MAX_POWER = 1.0;
    public static double kP = 0.005;
    public static double LATCH_SERVO_OPEN = 0.5;
    public static double LATCH_SERVO_CLOSED = 0;
    public static double PITCH_SERVO_MIN = 0;
    public static double PITCH_SERVO_MAX = 0.8;
    public static double TURRET_SERVO_MIN = 0.1;
    public static double TURRET_SERVO_MAX = 0.9;
    public static double FLOWER_HOOD_POS = 0.67;
    public static double FLOWER_WHEEL_SPEED = 300;
    public static Pose TURRET_TRANSLATION = new Pose(-3, -4);
}
