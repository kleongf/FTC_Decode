package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.biobuzz.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;

public class Intake extends Subsystem {
    private final DcMotorEx intakeMotor;
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConstants.INTAKE_MOTOR_NAME);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {
        intakeMotor.setPower(IntakeConstants.INTAKE_ON_SPEED);
        super.start();
    }

    public void intakeOn() {
        intakeMotor.setPower(IntakeConstants.INTAKE_ON_SPEED);
    }

    public void intakeOff() {
        intakeMotor.setPower(IntakeConstants.INTAKE_OFF_SPEED);
    }
}
