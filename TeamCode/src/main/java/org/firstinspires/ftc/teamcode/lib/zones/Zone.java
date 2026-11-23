package org.firstinspires.ftc.teamcode.lib.zones;

/** Axis-aligned field rectangle with optional per-axis targets (null = leave that axis manual). */
public class Zone {
    public final double cx, cy, width, height;
    public final Double targetX, targetY;
    public final HeadingTarget heading;

    public Zone(double cx, double cy, double width, double height,
                Double targetX, Double targetY, HeadingTarget heading) {
        this.cx = cx;
        this.cy = cy;
        this.width = width;
        this.height = height;
        this.targetX = targetX;
        this.targetY = targetY;
        this.heading = heading;
    }

    public static Zone pushX(double cx, double cy, double width, double height,
                             double targetX, HeadingTarget heading) {
        return new Zone(cx, cy, width, height, targetX, null, heading);
    }

    public static Zone pushY(double cx, double cy, double width, double height,
                             double targetY, HeadingTarget heading) {
        return new Zone(cx, cy, width, height, null, targetY, heading);
    }
}
