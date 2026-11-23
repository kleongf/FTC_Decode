package org.firstinspires.ftc.teamcode.biobuzz.opmode.teleop;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.biobuzz.CurrentRobot;
import org.firstinspires.ftc.teamcode.biobuzz.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.biobuzz.constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.biobuzz.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.biobuzz.constants.NectarShooterConstants;
import org.firstinspires.ftc.teamcode.biobuzz.constants.PollenShooterConstants;
import org.firstinspires.ftc.teamcode.biobuzz.constants.ShootingConstants;
import org.firstinspires.ftc.teamcode.lib.Alliance;
import org.firstinspires.ftc.teamcode.lib.robot.CommandScheduler;
import org.firstinspires.ftc.teamcode.lib.zones.IouZoneGuide;
import org.firstinspires.ftc.teamcode.util.SOTMUtil;

public class MainTeleop {
    public final CurrentRobot robot;
    private final Telemetry telemetry;
    private final Gamepad gamepad1, gamepad2;
    private Pose goalPose;
    private int sign = -1;
    private boolean doingFlower;

    public MainTeleop(Pose startPose, Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        robot = new CurrentRobot(hardwareMap, alliance);
        robot.drivetrain.follower.setStartingPose(startPose);
        robot.drivetrain.setGuide(new IouZoneGuide(
                FieldConstants.PUSH_ZONES,
                FieldConstants.ROBOT_WIDTH, FieldConstants.ROBOT_LENGTH,
                () -> DrivetrainConstants.MIN_ZONE_IOU));
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.goalPose = alliance == Alliance.BLUE ? FieldConstants.BLUE_SHOOT_POSE : FieldConstants.RED_SHOOT_POSE;
    }

    public void init() {
        robot.init();
    }

    public void init_loop() {
    }

    public void start() {
        robot.start();
    }

    public void periodic() {
        Pose currentPose = robot.drivetrain.follower.getPose();
        Vector currentVelocity = robot.drivetrain.follower.getVelocity();
        // 3 buttons
        if (gamepad1.xWasPressed()) {
            CommandScheduler.getInstance().schedule(
                    new ShootCommand(robot.transfer, robot.nectarShooter, robot.pollenShooter));
        }

        if (gamepad1.leftBumperWasPressed()) {
            robot.intake.intakeOff();
        }

        if (gamepad1.rightBumperWasPressed()) {
            robot.intake.intakeOn();
        }

        if (gamepad2.xWasPressed()) {
            sign *= -1;
            goalPose = new Pose(goalPose.getX(), goalPose.getY() + sign * 2 * FieldConstants.DISTANCE_TO_CENTER);
        }

        // gamepad 2
        if (gamepad2.yWasPressed()) {
            doingFlower = !doingFlower;
        }

        // logic
        if (doingFlower) {
            robot.pollenShooter.setWantedHoodPos(PollenShooterConstants.FLOWER_HOOD_POS);
            robot.pollenShooter.setWantedSpeed(PollenShooterConstants.FLOWER_WHEEL_SPEED);
            robot.pollenShooter.setWantedTurretAngle(Math.toRadians(0));
            robot.nectarShooter.setWantedHoodPos(NectarShooterConstants.FLOWER_HOOD_POS);
            robot.nectarShooter.setWantedSpeed(NectarShooterConstants.FLOWER_WHEEL_SPEED);
            robot.nectarShooter.setWantedTurretAngle(Math.toRadians(0));
        } else {
            SOTMUtil.ShooterOutputs pollenShooterOutputs = SOTMUtil.calculateShooterOutputs(
                    currentPose,
                    currentVelocity,
                    PollenShooterConstants.TURRET_TRANSLATION,
                    goalPose, ShootingConstants.pollenSpeedTable,
                    ShootingConstants.pollenHoodTable,
                    ShootingConstants.POLLEN_TOF_FUNCTION);

            SOTMUtil.ShooterOutputs nectarShooterOutputs = SOTMUtil.calculateShooterOutputs(
                    currentPose,
                    currentVelocity,
                    NectarShooterConstants.TURRET_TRANSLATION,
                    goalPose, ShootingConstants.nectarSpeedTable,
                    ShootingConstants.nectarHoodTable,
                    ShootingConstants.NECTAR_TOF_FUNCTION);

            robot.pollenShooter.setWantedSpeed(pollenShooterOutputs.speed);
            robot.pollenShooter.setWantedHoodPos(pollenShooterOutputs.hoodPos);
            robot.pollenShooter.setWantedTurretAngle(pollenShooterOutputs.turretAngle);

            robot.nectarShooter.setWantedSpeed(nectarShooterOutputs.speed);
            robot.nectarShooter.setWantedHoodPos(nectarShooterOutputs.hoodPos);
            robot.nectarShooter.setWantedTurretAngle(nectarShooterOutputs.turretAngle);
        }

        robot.drivetrain.driveManual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        robot.periodic();

        telemetry.addData("Pose", currentPose);
        telemetry.addData("Velocity", currentVelocity);
        telemetry.addData("Guide", robot.drivetrain.getGuideTargets());
        telemetry.update();
    }
}
