package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Intake;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.NectarShooter;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.PollenShooter;
import org.firstinspires.ftc.teamcode.biobuzz.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.lib.Alliance;
import org.firstinspires.ftc.teamcode.lib.robot.Robot;

public class CurrentRobot extends Robot {
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Transfer transfer;
    public final PollenShooter pollenShooter;
    public final NectarShooter nectarShooter;

    public CurrentRobot(HardwareMap hardwareMap, Alliance alliance) {
        drivetrain = new Drivetrain(hardwareMap, alliance);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        pollenShooter = new PollenShooter(hardwareMap);
        nectarShooter = new NectarShooter(hardwareMap);
    }

    @Override
    public void init() {
        drivetrain.init();
        intake.init();
        transfer.init();
        pollenShooter.init();
        nectarShooter.init();
    }

    @Override
    public void start() {
        drivetrain.start();
        intake.start();
        transfer.start();
        pollenShooter.start();
        nectarShooter.start();
    }

    @Override
    public void periodic() {
        super.periodic();

        drivetrain.periodic();
        intake.periodic();
        transfer.periodic();
        pollenShooter.periodic();
        nectarShooter.periodic();
    }
}
