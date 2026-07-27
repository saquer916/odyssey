package org.firstinspires.ftc.teamcode.odyssey.tests;

import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;
import org.junit.Test;
import static org.junit.Assert.*;


public class BezierTangentTest {
    private static final double EPS = 1e-9;

    private BezierCurve sample() {
        return new BezierCurve(
                new Vector2d(0, 0), new Vector2d(0, 3),
                new Vector2d(6, 3), new Vector2d(6, 0)
        );
    }

    @Test
    public void straightUp() {
        Vector2d tangent = sample().getTangentVector(0);
        assertEquals(0, tangent.getX(), EPS);
        assertEquals(9, tangent.getY(), EPS);
    }

    @Test
    public void straightDown() {
        Vector2d tangent = sample().getTangentVector(1);
        assertEquals(0, tangent.getX(), EPS);
        assertEquals(-9, tangent.getY(), EPS);
    }

    @Test
    public void horizontal() {
        Vector2d tangent = sample().getTangentVector(0.5);
        assertEquals(9, tangent.getX(), EPS);
        assertEquals(0, tangent.getY(), EPS);
    }

    @Test
    public void testAngleStraightUp() {
        assertEquals(Math.PI/2, sample().getTangentVector(0).getAngleFromCur(), EPS); // redundant but who cares
        assertEquals(Math.PI/2, sample().getTangentAngle(0), EPS);
    }

    @Test
    public void testAngleStraightDown() {
        assertEquals(0-(Math.PI/2), sample().getTangentVector(1).getAngleFromCur(), EPS);
        assertEquals(0-(Math.PI/2), sample().getTangentAngle(1), EPS);
    }

    @Test
    public void testAngleHorizontal() {
        assertEquals(0, sample().getTangentVector(0.5).getAngleFromCur(), EPS);
        assertEquals(0, sample().getTangentAngle(0.5), EPS);
    }
}
