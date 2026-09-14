package org.firstinspires.ftc.teamcode.biobuzz.opmode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.biobuzz.CurrentRobot;
import org.firstinspires.ftc.teamcode.biobuzz.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.biobuzz.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.biobuzz.constants.NectarShooterConstants;
import org.firstinspires.ftc.teamcode.biobuzz.constants.PollenShooterConstants;
import org.firstinspires.ftc.teamcode.biobuzz.constants.ShootingConstants;
import org.firstinspires.ftc.teamcode.lib.Alliance;
import org.firstinspires.ftc.teamcode.lib.fsm.State;
import org.firstinspires.ftc.teamcode.lib.fsm.StateMachine;
import org.firstinspires.ftc.teamcode.lib.fsm.Transition;
import org.firstinspires.ftc.teamcode.lib.robot.Command;
import org.firstinspires.ftc.teamcode.lib.robot.CommandScheduler;
import org.firstinspires.ftc.teamcode.util.SOTMUtil;

@Autonomous(name = "Red Close 2 Tip", group = "!")
public class ExampleAuto extends OpMode {
    private CurrentRobot robot;
    private StateMachine stateMachine;
    private Command shootCommand;

    private final Pose startPose = FieldConstants.RED_CLOSE_START_POSE;
    private final Pose scorePose = FieldConstants.RED_SHOOT_POSE;
    private final Pose parkPose = new Pose(10, 100, Math.toRadians(90));

    private PathChain intakeCorner;
    private PathChain intakePile;
    private PathChain park;

    private void buildPaths() {
        Follower follower = robot.drivetrain.follower;

        intakeCorner = follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(startPose, new Pose(9, 144-9))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        intakePile = follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(new Pose(9, 144-9), startPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new Path(
                                new BezierLine(startPose, parkPose)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), parkPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        // this auto only drives via followPath/TurnToHeadingCommand, never manual joystick
        // input, so the alliance-dependent heading offset (see Drivetrain) has no effect here
        robot = new CurrentRobot(hardwareMap, Alliance.BLUE);
        robot.drivetrain.follower.setStartingPose(startPose);
        robot.init();
        buildPaths();

        stateMachine = new StateMachine(
                new State()
                        .onEnter(() -> robot.intake.intakeOn())
                        .transition(new Transition(() -> robot.nectarShooter.getSpeedError() < 30 && robot.pollenShooter.getSpeedError() < 30)),
                new State()
                        .onEnter(() -> {
                            shootCommand = new ShootCommand(robot.transfer, robot.nectarShooter, robot.pollenShooter);
                            CommandScheduler.getInstance().schedule(shootCommand);
                        })
                        .transition(new Transition(() -> shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.intake.intakeOn();
                            robot.drivetrain.follower.followPath(intakeCorner, true);
                        })
                        .transition(new Transition(() -> !robot.drivetrain.follower.isBusy())),
                new State()
                        .maxTime(15_000),
                new State()
                        .onEnter(() -> {
                            shootCommand = new ShootCommand(robot.transfer, robot.nectarShooter, robot.pollenShooter);
                            CommandScheduler.getInstance().schedule(shootCommand);
                        })
                        .transition(new Transition(() -> shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.intake.intakeOn();
                            robot.drivetrain.follower.followPath(intakePile, true);
                        })
                        .transition(new Transition(() -> !robot.drivetrain.follower.isBusy())),
                new State()
                        .maxTime(1000),
                new State()
                        .onEnter(() -> {
                            shootCommand = new ShootCommand(robot.transfer, robot.nectarShooter, robot.pollenShooter);
                            CommandScheduler.getInstance().schedule(shootCommand);
                        })
                        .transition(new Transition(() -> shootCommand.isFinished())),
                new State()
                        .onEnter(() -> {
                            robot.intake.intakeOn();
                            robot.drivetrain.follower.followPath(park, true);
                        })
                        .transition(new Transition(() -> !robot.drivetrain.follower.isBusy()))
        );
    }

    @Override
    public void start() {
        stateMachine.start();
        robot.start();
    }

    @Override
    public void loop() {
        Pose currentPose = robot.drivetrain.follower.getPose();
        Vector currentVelocity = robot.drivetrain.follower.getVelocity();
        Pose goalPose = FieldConstants.RED_SHOOT_POSE;

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

        stateMachine.periodic();
        robot.periodic();

        telemetry.addData("Pose", robot.drivetrain.follower.getPose());
        telemetry.update();
    }
}
