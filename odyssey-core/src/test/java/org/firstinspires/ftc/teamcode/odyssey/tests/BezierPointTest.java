package org.firstinspires.ftc.teamcode.odyssey.tests;
import static org.junit.Assert.*;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;
import org.junit.Test;
public class BezierPointTest {
    private static final double EPS = 1e-9;

    private BezierCurve sample() {
        return new BezierCurve(
                new Vector2d(0, 0), new Vector2d(0, 3),
                new Vector2d(6, 3), new Vector2d(6, 0)
        );
    }

    @Test
    public void t0Equalsp0() {
        Vector2d point = sample().getPoint(0);
        assertEquals(0, point.getX(), EPS);
        assertEquals(0, point.getY(), EPS);
    }

    @Test
    public void t1Equalsp3() {
        Vector2d point = sample().getPoint(1);
        assertEquals(6, point.getX(), EPS);
        assertEquals(0, point.getY(), EPS);
    }

    @Test
    public void midpoint() {
        Vector2d point = sample().getPoint(0.5);
        assertEquals(3.0, point.getX(), EPS);
        assertEquals(2.25, point.getY(), EPS);
    }

    @Test
    public void degenerateCurveIsAPoint() {
        Vector2d samePoint = new Vector2d(5, 5);
        BezierCurve bez = new BezierCurve(samePoint, samePoint, samePoint, samePoint);
        assertEquals(5, bez.getPoint(0.72).getX(), EPS); // random t val to test
        assertEquals(5, bez.getPoint(0.72).getY(), EPS);
    }
}
