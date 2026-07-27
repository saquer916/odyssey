package org.firstinspires.ftc.teamcode.odyssey.tests;

import static org.junit.Assert.*;
import org.junit.Test;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;

public class BezierLengthDistAndTTest {
    private static final double EPS = 1e-9;
    private static final double NUM_EPS = 1e-6; // numerical results

    private BezierCurve arch() {
        return new BezierCurve(
                new Vector2d(0, 0), new Vector2d(0, 3),
                new Vector2d(6, 3), new Vector2d(6, 0)
        );
    }

    private BezierCurve straightLine() {
        return new BezierCurve(
                new Vector2d(0, 0), new Vector2d(1, 0),
                new Vector2d(2, 0), new Vector2d(3, 0)
        );
    }

    private BezierCurve point() {
        Vector2d p = new Vector2d(5, 5);
        return new BezierCurve(p, p, p, p);
    }

    @Test
    public void lengthFromZeroIsZero() {
        assertEquals(0.0, arch().getLength(0), EPS);
    }

    @Test
    public void straightLineIsExact() {
        assertEquals(3.0, straightLine().getLength(1), NUM_EPS);
    }

    @Test
    public void straightLineHalfIsExact() {
        assertEquals(1.5, straightLine().getLength(0.5), NUM_EPS);
    }

    @Test
    public void archIsLongerThanItsChord() {
        assertTrue(arch().getLength(1) > 6.0);
    }

    @Test
    public void lengthIncreasesWithB() {
        BezierCurve c = arch();
        assertTrue(c.getLength(0.5) > 0);
        assertTrue(c.getLength(0.5) < c.getLength(1));
    }

    @Test
    public void degenerateCurveHasZeroLength() {
        assertEquals(0.0, point().getLength(1), NUM_EPS);
    }

    @Test
    public void distanceZeroGivesTZero() {
        assertEquals(0.0, arch().getTAtDistance(0), EPS);
    }

    @Test
    public void fullDistanceGivesTOne() {
        BezierCurve c = arch();
        assertEquals(1.0, c.getTAtDistance(c.getLength(1)), EPS);
    }

    @Test
    public void distanceEqualsTAtSamePoint() {
        BezierCurve c = arch();
        double half = c.getLength(1) / 2.0;
        double t = c.getTAtDistance(half);
        assertEquals(half, c.getLength(t), NUM_EPS);
    }

    @Test
    public void straightLineInversionIsExact() {
        assertEquals(0.5, straightLine().getTAtDistance(1.5), NUM_EPS);
    }

    @Test
    public void distanceBelowZeroReturnsZero() {
        assertEquals(0.0, arch().getTAtDistance(-5), EPS);
    }

    @Test
    public void distanceAboveLengthReturnsOne() {
        assertEquals(1.0, arch().getTAtDistance(10000), EPS);
    }
}
