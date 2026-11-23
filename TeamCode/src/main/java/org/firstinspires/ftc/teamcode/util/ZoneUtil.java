package org.firstinspires.ftc.teamcode.util;

public class ZoneUtil {
    private static boolean lineLine(
            double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
        double uA =
                ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3))
                        / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
        double uB =
                ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3))
                        / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
        return uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1;
    }

    private static boolean pointRect(double x, double y, double cx, double cy, double w, double h) {
        return Math.abs(x - cx) <= w / 2 && Math.abs(y - cy) <= h / 2;
    }

    private static boolean lineRect(
            double x1, double y1, double x2, double y2, double cx, double cy, double w, double h) {
        if (pointRect(x1, y1, cx, cy, w, h) || pointRect(x2, y2, cx, cy, w, h)) {
            return true;
        }
        double xMin = cx - w / 2, xMax = cx + w / 2;
        double yMin = cy - h / 2, yMax = cy + h / 2;
        boolean topBottom =
                lineLine(x1, y1, x2, y2, xMin, yMin, xMax, yMin)
                        || lineLine(x1, y1, x2, y2, xMin, yMax, xMax, yMax);
        boolean leftRight =
                lineLine(x1, y1, x2, y2, xMin, yMin, xMin, yMax)
                        || lineLine(x1, y1, x2, y2, xMax, yMin, xMax, yMax);
        return topBottom || leftRight;
    }

    /** Corners (counter-clockwise) of a w x h rectangle centered at (cx, cy), rotated by heading. */
    public static double[][] rectCorners(double cx, double cy, double w, double h, double heading) {
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double[][] local = {{w / 2, h / 2}, {-w / 2, h / 2}, {-w / 2, -h / 2}, {w / 2, -h / 2}};
        double[][] corners = new double[4][2];
        for (int i = 0; i < 4; i++) {
            corners[i][0] = cx + local[i][0] * cos - local[i][1] * sin;
            corners[i][1] = cy + local[i][0] * sin + local[i][1] * cos;
        }
        return corners;
    }

    /** Sutherland–Hodgman clip of a convex polygon against an axis-aligned rectangle. */
    public static double[][] clipToAxisRect(double[][] poly, double cx, double cy, double w, double h) {
        poly = clipHalfPlane(poly, 0, cx - w / 2, true);
        poly = clipHalfPlane(poly, 0, cx + w / 2, false);
        poly = clipHalfPlane(poly, 1, cy - h / 2, true);
        poly = clipHalfPlane(poly, 1, cy + h / 2, false);
        return poly;
    }

    /** Keeps the part of {@code poly} where coordinate {@code axis} is >= bound (keepAbove) or <= bound. */
    private static double[][] clipHalfPlane(double[][] poly, int axis, double bound, boolean keepAbove) {
        java.util.List<double[]> out = new java.util.ArrayList<>();
        int n = poly.length;
        for (int i = 0; i < n; i++) {
            double[] cur = poly[i];
            double[] prev = poly[(i + n - 1) % n];
            boolean curIn = keepAbove ? cur[axis] >= bound : cur[axis] <= bound;
            boolean prevIn = keepAbove ? prev[axis] >= bound : prev[axis] <= bound;
            if (curIn != prevIn) {
                double t = (bound - prev[axis]) / (cur[axis] - prev[axis]);
                out.add(new double[] {
                        prev[0] + t * (cur[0] - prev[0]),
                        prev[1] + t * (cur[1] - prev[1])
                });
            }
            if (curIn) out.add(cur);
        }
        return out.toArray(new double[0][]);
    }

    /** Shoelace area (always non-negative). */
    public static double polygonArea(double[][] poly) {
        double sum = 0;
        for (int i = 0; i < poly.length; i++) {
            double[] a = poly[i], b = poly[(i + 1) % poly.length];
            sum += a[0] * b[1] - b[0] * a[1];
        }
        return Math.abs(sum) / 2;
    }

    /**
     * Intersection-over-union of the robot footprint (rW x rL centered at (rx, ry), rotated by
     * rHeading) and an axis-aligned zone (zw x zh centered at (zcx, zcy)).
     */
    public static double iou(double rx, double ry, double rHeading, double rW, double rL,
                             double zcx, double zcy, double zw, double zh) {
        // heading 0 faces +x, so length runs along the local x axis
        double[][] robot = rectCorners(rx, ry, rL, rW, rHeading);
        double inter = polygonArea(clipToAxisRect(robot, zcx, zcy, zw, zh));
        double union = rW * rL + zw * zh - inter;
        return union <= 0 ? 0 : inter / union;
    }
}
