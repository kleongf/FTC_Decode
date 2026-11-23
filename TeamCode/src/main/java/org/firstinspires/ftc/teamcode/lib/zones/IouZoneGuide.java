package org.firstinspires.ftc.teamcode.lib.zones;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.util.ZoneUtil;

import java.util.List;
import java.util.function.DoubleSupplier;

/** Guides toward the targets of the zone with the highest IOU against the rotated robot footprint. */
public class IouZoneGuide extends DriveGuide {
    protected final List<Zone> zones;
    protected final double robotWidth, robotLength;
    protected final DoubleSupplier minIou;

    public IouZoneGuide(List<Zone> zones, double robotWidth, double robotLength, DoubleSupplier minIou) {
        this.zones = zones;
        this.robotWidth = robotWidth;
        this.robotLength = robotLength;
        this.minIou = minIou;
    }

    @Override
    public GuideTargets getTargets(Pose pose) {
        Zone zone = selectZone(pose);
        if (zone == null) return GuideTargets.NONE;
        return new GuideTargets(zone.targetX, zone.targetY, zone.heading.target(pose.getHeading()));
    }

    /** The zone with the highest IOU above the minimum, or null. */
    protected Zone selectZone(Pose pose) {
        Zone best = null;
        double bestIou = minIou.getAsDouble();
        for (Zone zone : zones) {
            double iou = iou(pose, zone);
            if (iou > bestIou) {
                bestIou = iou;
                best = zone;
            }
        }
        return best;
    }

    protected double iou(Pose pose, Zone zone) {
        return ZoneUtil.iou(pose.getX(), pose.getY(), pose.getHeading(), robotWidth, robotLength,
                zone.cx, zone.cy, zone.width, zone.height);
    }
}
