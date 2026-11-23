package org.firstinspires.ftc.teamcode.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class ZoneUtilTest {
    private static final double EPS = 1e-9;

    @Test
    public void identicalBoxesHaveIouOne() {
        assertEquals(1, ZoneUtil.iou(10, 10, 0, 4, 4, 10, 10, 4, 4), EPS);
    }

    @Test
    public void disjointBoxesHaveIouZero() {
        assertEquals(0, ZoneUtil.iou(0, 0, 0, 4, 4, 100, 100, 4, 4), EPS);
    }

    @Test
    public void halfOverlapHasIouOneThird() {
        // 4x4 boxes offset by 2 in x: intersection 8, union 24
        assertEquals(1 / 3d, ZoneUtil.iou(2, 0, 0, 4, 4, 0, 0, 4, 4), EPS);
    }

    @Test
    public void rotatedSquareOverSameSquare() {
        // unit-area squares, one rotated 45°: intersection is a regular octagon of area 2(sqrt2 - 1)
        double inter = 2 * (Math.sqrt(2) - 1);
        assertEquals(inter / (2 - inter), ZoneUtil.iou(0, 0, Math.PI / 4, 1, 1, 0, 0, 1, 1), EPS);
    }

    @Test
    public void rotationChangesOverlapWithThinZone() {
        // 2x10 robot (length along heading) inside a 2x10 vertical zone only when turned 90°
        double turned = ZoneUtil.iou(0, 0, Math.PI / 2, 2, 10, 0, 0, 2, 10);
        double straight = ZoneUtil.iou(0, 0, 0, 2, 10, 0, 0, 2, 10);
        assertEquals(1, turned, EPS);
        assertEquals(4 / 36d, straight, EPS);
    }
}
