package org.firstinspires.ftc.teamcode.lib.zones;

import com.pedropathing.geometry.Pose;

/** Extension point for zone logic: maps the robot pose to the setpoints the drivetrain is guided toward. */
public abstract class DriveGuide {
    public abstract GuideTargets getTargets(Pose pose);
}
