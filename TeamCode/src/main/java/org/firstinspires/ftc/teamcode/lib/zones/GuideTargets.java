package org.firstinspires.ftc.teamcode.lib.zones;

/** Field-frame setpoints a {@link DriveGuide} wants; null means that axis stays manual. */
public class GuideTargets {
    public static final GuideTargets NONE = new GuideTargets(null, null, null);

    public final Double x, y, heading;

    public GuideTargets(Double x, Double y, Double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    @Override
    public String toString() {
        return String.format("x=%s y=%s heading=%s", x, y,
                heading == null ? null : String.format("%.1f°", Math.toDegrees(heading)));
    }
}
