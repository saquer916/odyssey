package org.firstinspires.ftc.teamcode.odyssey.tests;

import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.odyssey.utils.MathUtils.normalizeAngle;

import org.firstinspires.ftc.teamcode.odyssey.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.junit.Test;


public class GeometryTest {

    private static final double EPS = 1e-6;


    @Test
    public void straightAhead() {
        Pose2d pose = new Pose2d(2, 3, 0);
        Vector2d r = pose.toRobotFrame(new Vector2d(5, 3));
        assertEquals(3, r.getX(), EPS);
        assertEquals(0, r.getY(), EPS);
    }

    @Test
    public void turned90_targetAhead() {
        Pose2d pose = new Pose2d(0, 0, Math.PI / 2);
        Vector2d r = pose.toRobotFrame(new Vector2d(0, 4));
        assertEquals(4, r.getX(), EPS);
        assertEquals(0, r.getY(), EPS);
    }

    @Test
    public void turned90_targetToSide() {
        Pose2d pose = new Pose2d(0, 0, Math.PI / 2);
        Vector2d r = pose.toRobotFrame(new Vector2d(-3, 0));
        assertEquals(0, r.getX(), EPS);
        assertEquals(3, r.getY(), EPS);
    }

    @Test
    public void rotate90() {
        Vector2d r = new Vector2d(1, 0).rotateVector(Math.PI / 2);
        assertEquals(0, r.getX(), EPS);
        assertEquals(1, r.getY(), EPS);
    }

    @Test
    public void rotate180() {
        Vector2d r = new Vector2d(1, 0).rotateVector(Math.PI);
        assertEquals(-1, r.getX(), EPS);
        assertEquals(0, r.getY(), EPS);
    }

    @Test
    public void addVectors() {
        Vector2d r = new Vector2d(2, 1).add(new Vector2d(3, 4));
        assertEquals(5, r.getX(), EPS);
        assertEquals(5, r.getY(), EPS);
    }

    @Test
    public void subtractVectors() {
        Vector2d r = new Vector2d(5, 3).subtract(new Vector2d(2, 1));
        assertEquals(3, r.getX(), EPS);
        assertEquals(2, r.getY(), EPS);
    }

    @Test
    public void magnitude345() {
        assertEquals(5, new Vector2d(3, 4).getMagnitude(), EPS);
    }

    @Test
    public void magnitudeNegative() {
        assertEquals(5, new Vector2d(-3, 4).getMagnitude(), EPS);
    }

    @Test
    public void relativeTo_workedExample() {
        Pose2d robot = new Pose2d(2, 2, Math.PI / 2);
        Pose2d target = new Pose2d(4, 5, Math.PI);
        Pose2d rel = target.relativeTo(robot);
        assertEquals(3, rel.getX(), EPS);
        assertEquals(-2, rel.getY(), EPS);
        assertEquals(Math.PI / 2, rel.getHeading(), EPS);
    }

    @Test
    public void relativeTo_negativeHeadingTurn() {
        Pose2d robot = new Pose2d(0, 0, Math.toRadians(-25));
        Pose2d target = new Pose2d(0, 0, Math.toRadians(90));
        Pose2d rel = target.relativeTo(robot);
        assertEquals(Math.toRadians(115), rel.getHeading(), EPS);
    }

    @Test
    public void constructorNormalizesHeading() {
        Pose2d p = new Pose2d(0, 0, 3 * Math.PI / 2);
        assertEquals(-Math.PI / 2, p.getHeading(), EPS);
    }


    @Test
    public void rotationPreservesLength() {
        Vector2d v = new Vector2d(3, 4);
        assertEquals(5, v.rotateVector(1.2345).getMagnitude(), EPS);
    }

    @Test
    public void roundTrip() {
        Pose2d pose = new Pose2d(2, 2, Math.PI / 3);
        Vector2d original = new Vector2d(7, 1);
        Vector2d back = pose.toFieldFrame(pose.toRobotFrame(original));
        assertEquals(original.getX(), back.getX(), EPS);
        assertEquals(original.getY(), back.getY(), EPS);
    }

    @Test
    public void normalizeStaysInRange() {
        Pose2d p = new Pose2d(0, 0, 0);
        double[] inputs = {10, -10, 100, -100, 7, -7};
        for (double a : inputs) {
            double n = normalizeAngle(a);
            assertTrue("out of range: " + n, n >= -Math.PI - EPS && n <= Math.PI + EPS);
        }
    }

    @Test
    public void addCommutative() {
        Vector2d a = new Vector2d(2, 1);
        Vector2d b = new Vector2d(3, 4);
        assertEquals(a.add(b).getX(), b.add(a).getX(), EPS);
        assertEquals(a.add(b).getY(), b.add(a).getY(), EPS);
    }


    @Test
    public void toRobotFrame_identity() {
        Pose2d pose = new Pose2d(0, 0, 0);
        Vector2d r = pose.toRobotFrame(new Vector2d(7, 2));
        assertEquals(7, r.getX(), EPS);
        assertEquals(2, r.getY(), EPS);
    }

    @Test
    public void relativeTo_self() {
        Pose2d p = new Pose2d(3, 4, 1);
        Pose2d rel = p.relativeTo(p);
        assertEquals(0, rel.getX(), EPS);
        assertEquals(0, rel.getY(), EPS);
        assertEquals(0, rel.getHeading(), EPS);
    }

    @Test
    public void zeroVectorMagnitude() {
        assertEquals(0, new Vector2d(0, 0).getMagnitude(), EPS);
    }

    @Test
    public void normalizeBoundaries() {
        Pose2d p = new Pose2d(0, 0, 0);
        assertEquals(0, normalizeAngle(2 * Math.PI), EPS);
        assertEquals(Math.PI, normalizeAngle(Math.PI), EPS);
        assertEquals(0, normalizeAngle(0), EPS);
    }
}