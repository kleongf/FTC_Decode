package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.biobuzz.constants.ShootingConstants;
import org.firstinspires.ftc.teamcode.lib.math.LookupTable;

import java.util.function.Function;

public class SOTMUtil {
    public static class ShooterOutputs {
        public double speed;
        public double hoodPos;
        public double turretAngle;
        public ShooterOutputs(double speed, double hoodPos, double turretAngle) {
            this.speed = speed;
            this.hoodPos = hoodPos;
            this.turretAngle = turretAngle;
        }
    }

    private static Pose vectorToPose(Vector v) {
        return new Pose(v.getXComponent(), v.getYComponent());
    }
    public static double calculateTOF(
            Function<Double, Double> tofFunction,
            Pose robotPose,
            Pose targetPose,
            Vector robotVelocity
    ) {
        double runningX = robotPose.getX();
        double runningY = robotPose.getY();

        double tof = 0;

        for (int i = 0; i < ShootingConstants.MAX_TOF_ITERATIONS; i++) {
            // get distance
            double dx = targetPose.getX() - runningX;
            double dy = targetPose.getY() - runningY;
            double distance = Math.hypot(dx, dy);
            // get tof
            tof = tofFunction.apply(distance);
            // update running tof
            runningX = robotPose.getX() + robotVelocity.getXComponent() * tof;
            runningY = robotPose.getY() + robotVelocity.getYComponent() * tof;
        }

        return tof;
    }
    public static ShooterOutputs calculateShooterOutputs(Pose robotPose, Vector robotVelocity, Pose translation, Pose goalPose, LookupTable speedTable, LookupTable hoodPosTable, Function<Double, Double> tofFunction) {
        Pose turretPose = robotPose.plus(translation.rotate(robotPose.getHeading(), false)).minus(vectorToPose(robotVelocity.times(calculateTOF(tofFunction, robotPose, goalPose, robotVelocity))));

        double dx = goalPose.getX() - turretPose.getX();
        double dy = goalPose.getY() - turretPose.getY();
        double distance = goalPose.distanceFrom(turretPose);

        return new ShooterOutputs(
                speedTable.getValue(distance),
                hoodPosTable.getValue(distance),
                Math.atan2(dy, dx) - turretPose.getHeading() + Math.toRadians(90)
        );
    }
}
