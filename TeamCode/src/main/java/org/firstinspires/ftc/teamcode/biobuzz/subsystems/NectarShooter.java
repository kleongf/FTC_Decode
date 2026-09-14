package org.firstinspires.ftc.teamcode.biobuzz.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.biobuzz.constants.NectarShooterConstants;
import org.firstinspires.ftc.teamcode.lib.controllers.PIDFController;
import org.firstinspires.ftc.teamcode.lib.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class NectarShooter extends Subsystem {
    private final DcMotorEx shooterMotor;
    private final Servo pitchServo;
    private final Servo latchServo;
    private final Servo turretServo;
    private final PIDFController speedController;
    private double wantedSpeed;
    private double wantedHoodPos;
    private double wantedTurretAngle;
    private double currentSpeed;

    public NectarShooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, NectarShooterConstants.SHOOTER_MOTOR_NAME);
        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pitchServo = hardwareMap.get(Servo.class, NectarShooterConstants.PITCH_SERVO_NAME);
        latchServo = hardwareMap.get(Servo.class, NectarShooterConstants.LATCH_SERVO_NAME);
        turretServo = hardwareMap.get(Servo.class, NectarShooterConstants.TURRET_SERVO_NAME);

        speedController = new PIDFController(NectarShooterConstants.kP, 0, 0, 0);
    }
    private double calculateWantedTurretPos(double wantedAngle) {
        double t = MathUtil.inverseLerp(-Math.PI, Math.PI, MathUtil.angleModulusSigned(wantedAngle));
        return MathUtil.lerp(NectarShooterConstants.TURRET_SERVO_MIN,
                NectarShooterConstants.TURRET_SERVO_MAX, t);
    }

    @Override
    public void periodic() {
        super.periodic();

        currentSpeed = shooterMotor.getVelocity();
        double power = speedController.calculate(currentSpeed, wantedSpeed);
        shooterMotor.setPower(MathUtil.clamp(power, 0, NectarShooterConstants.MAX_POWER));
        pitchServo.setPosition(MathUtil.clamp(wantedHoodPos, NectarShooterConstants.PITCH_SERVO_MIN, NectarShooterConstants.PITCH_SERVO_MAX));
        turretServo.setPosition(MathUtil.clamp(calculateWantedTurretPos(wantedTurretAngle), NectarShooterConstants.TURRET_SERVO_MIN, NectarShooterConstants.TURRET_SERVO_MAX));
    }

    public void openLatch() {
        latchServo.setPosition(NectarShooterConstants.LATCH_SERVO_OPEN);
    }

    public void closeLatch() {
        latchServo.setPosition(NectarShooterConstants.LATCH_SERVO_CLOSED);
    }

    public void setWantedSpeed(double speed) {
        wantedSpeed = speed;
    }

    public double getWantedSpeed(double speed) {
        return wantedSpeed;
    }

    public void setWantedTurretAngle(double angle) {
        wantedTurretAngle = angle;
    }

    public double getWantedTurretAngle(double angle) {
        return wantedTurretAngle;
    }

    public void setWantedHoodPos(double pos) {
        wantedHoodPos = pos;
    }

    public double getWantedHoodPos() {
        return wantedHoodPos;
    }

    public double getSpeedError() {
        return wantedSpeed - currentSpeed;
    }
}
