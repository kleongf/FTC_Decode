package org.firstinspires.ftc.teamcode.lib.zones;

/** Resolves a zone's heading target from the robot's current heading; null leaves heading manual. */
@FunctionalInterface
public interface HeadingTarget {
    Double target(double currentHeading);

    HeadingTarget FREE = heading -> null;

    static HeadingTarget nearest90() {
        // integer quarter-turns in [0, 4) keep the result in [0, 2π)
        return heading -> Math.floorMod(Math.round(heading / (Math.PI / 2)), 4) * (Math.PI / 2);
    }

    static HeadingTarget fixed(double radians) {
        return heading -> radians;
    }
}
