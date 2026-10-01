package org.firstinspires.ftc.teamcode.odyssey.tests;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.odyssey.control.PIDController;
import org.firstinspires.ftc.teamcode.odyssey.follower.DriveSignal;
import org.firstinspires.ftc.teamcode.odyssey.follower.Follower;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.localization.Localizer;
import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;
import org.firstinspires.ftc.teamcode.odyssey.path.Path;
import org.firstinspires.ftc.teamcode.odyssey.path.VelocityProfile;
import org.firstinspires.ftc.teamcode.odyssey.path.heading.ConstantInterpolator;
import org.junit.Test;

// p0 == p1 / p2 == p3 is the usual way to write a straight line, and makes B'(t) exactly zero at
// that endpoint. FirstTest's path is one of these.
public class DoubledControlPointTest {

    private static final double EPS = 1e-9;

    // Same shape as FirstTest: 100 mm straight along +Y.
    private BezierCurve doubledLine() {
        return new BezierCurve(
                new Vector2d(0, 0), new Vector2d(0, 0),
                new Vector2d(0, 100), new Vector2d(0, 100), new ConstantInterpolator(0));
    }

    private BezierCurve doubledStart() {
        return new BezierCurve(
                new Vector2d(0, 0), new Vector2d(0, 0),
                new Vector2d(100, 0), new Vector2d(100, 100), new ConstantInterpolator(0));
    }

    private BezierCurve doubledEnd() {
        return new BezierCurve(
                new Vector2d(0, 0), new Vector2d(0, 100),
                new Vector2d(100, 100), new Vector2d(100, 100), new ConstantInterpolator(0));
    }

    private static Localizer fixedAt(final Pose2d pose) {
        return new Localizer() {
            @Override public Pose2d getPose() { return pose; }
            @Override public void update() {}
        };
    }

    private static void assertVector(double x, double y, Vector2d actual) {
        assertEquals(x, actual.getX(), EPS);
        assertEquals(y, actual.getY(), EPS);
    }

    private static void assertNoNaN(DriveSignal signal) {
        assertFalse("forward velocity is NaN", Double.isNaN(signal.getForwardVelocity()));
        assertFalse("strafe velocity is NaN", Double.isNaN(signal.getStrafeVelocity()));
        assertFalse("forward accel is NaN", Double.isNaN(signal.getForwardAcceleration()));
        assertFalse("strafe accel is NaN", Double.isNaN(signal.getStrafeAcceleration()));
        assertFalse("turn is NaN", Double.isNaN(signal.getTurn()));
    }


    @Test public void curvatureAtDoubledEndpointsIsZeroNotNaN() {
        assertEquals(0.0, doubledLine().getCurvature(0), EPS);
        assertEquals(0.0, doubledLine().getCurvature(1), EPS);
    }

    @Test public void centripetalAtDoubledEndpointsIsZeroNotNaN() {
        assertVector(0, 0, doubledLine().getCentripetalVector(0));
        assertVector(0, 0, doubledLine().getCentripetalVector(1));
    }

    @Test public void directionAtDoubledEndpointsFollowsTheLine() {
        assertVector(0, 1, doubledLine().getTangentDirection(0));
        assertVector(0, 1, doubledLine().getTangentDirection(1));
    }

    @Test public void directionAtDoubledStartPointsAtP2() {
        assertVector(1, 0, doubledStart().getTangentDirection(0));
    }

    @Test public void directionAtDoubledEndComesFromP1() {
        assertVector(1, 0, doubledEnd().getTangentDirection(1));
    }

    @Test public void directionIsNormalizedTangentAwayFromEndpoints() {
        BezierCurve c = doubledStart();
        Vector2d expected = c.getTangentVector(0.3).normalize();
        assertVector(expected.getX(), expected.getY(), c.getTangentDirection(0.3));
    }

    @Test public void tangentAngleAtDoubledStartIsNotSnappedToZero() {
        // was atan2(0, 0) == 0, which TangentInterpolator would turn the robot towards
        assertEquals(Math.PI / 2, doubledLine().getTangentAngle(0), EPS);
    }

    @Test public void pathTangentAtStartOfDoubledLineIsAUnitVector() {
        assertVector(0, 1, new Path(doubledLine()).getTangentFromPathDistance(0));
    }


    private Follower followerAt(Path path, Pose2d pose) {
        VelocityProfile profile = new VelocityProfile(path, 1900, 3000, 3000, 2500, 1.25);
        return new Follower(path, fixedAt(pose), profile,
                new PIDController(0, 0.25, 0, 0.001), new PIDController(0, 2.0, 0, 0.001), 110, 500);
    }

    @Test public void followerSignalAtStartOfDoubledLineHasNoNaN() {
        Path path = new Path(doubledLine());
        assertNoNaN(followerAt(path, new Pose2d(0, 0, 0)).update(0));
    }

    @Test public void followerSignalAtEndOfDoubledLineHasNoNaN() {
        Path path = new Path(doubledLine());
        assertNoNaN(followerAt(path, new Pose2d(0, 100, 0)).update(0));
    }
}
