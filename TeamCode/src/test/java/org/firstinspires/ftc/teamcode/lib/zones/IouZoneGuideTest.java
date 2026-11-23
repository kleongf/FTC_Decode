package org.firstinspires.ftc.teamcode.lib.zones;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;

import com.pedropathing.geometry.Pose;

import org.junit.Test;

import java.util.Arrays;

public class IouZoneGuideTest {
    private static final double EPS = 1e-9;

    private final Zone left = Zone.pushX(43, 72, 10, 60, 39, HeadingTarget.nearest90());
    private final Zone right = Zone.pushX(53, 72, 10, 60, 57, HeadingTarget.nearest90());
    private final IouZoneGuide guide = new IouZoneGuide(Arrays.asList(left, right), 18, 18, () -> 0.05);

    @Test
    public void picksZoneWithHigherIou() {
        assertSame(left, guide.selectZone(new Pose(42, 72, 0)));
        assertSame(right, guide.selectZone(new Pose(55, 72, 0)));
    }

    @Test
    public void noTargetsBelowMinimumIou() {
        GuideTargets targets = guide.getTargets(new Pose(10, 72, 0));
        assertSame(GuideTargets.NONE, targets);
    }

    @Test
    public void targetsComeFromSelectedZone() {
        GuideTargets targets = guide.getTargets(new Pose(42, 72, Math.toRadians(80)));
        assertEquals(39, targets.x, EPS);
        assertNull(targets.y);
        assertEquals(Math.PI / 2, targets.heading, EPS);
    }

    @Test
    public void nearest90WrapsNegativeHeadings() {
        assertEquals(Math.toRadians(270), HeadingTarget.nearest90().target(Math.toRadians(-100)), EPS);
        assertEquals(0, HeadingTarget.nearest90().target(Math.toRadians(-30)), EPS);
    }

    @Test
    public void freeHeadingLeavesHeadingManual() {
        assertNull(HeadingTarget.FREE.target(1.0));
    }
}
