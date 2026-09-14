package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.biobuzz.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;

public class Transfer extends Subsystem {
    private final DcMotorEx transferMotor;
    public Transfer(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, TransferConstants.TRANSFER_MOTOR_NAME);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {
        transferMotor.setPower(TransferConstants.TRANSFER_ON_SPEED);
        super.start();
    }

    public void transferOn() {
        transferMotor.setPower(TransferConstants.TRANSFER_ON_SPEED);
    }

    public void transferOff() {
        transferMotor.setPower(TransferConstants.TRANSFER_OFF_SPEED);
    }
}
