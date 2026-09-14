package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.lib.math.LookupTable;

public class SOTMUtil {
    public class ShooterOutputs {
        public double speed;
        public double hoodPos;
        public double turretAngle;
        public ShooterOutputs(double speed, double hoodPos, double turretAngle) {
            this.speed = speed;
            this.hoodPos = hoodPos;
            this.turretAngle = turretAngle;
        }
    }

    public ShooterOutputs calculateShooterOutputs(Pose turretPose, Pose goalPose, LookupTable speedTable, LookupTable hoodPosTable) {
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
