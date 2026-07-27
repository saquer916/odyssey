package org.firstinspires.ftc.teamcode.odyssey.tests;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;
import org.firstinspires.ftc.teamcode.odyssey.path.Path;
import org.junit.Test;

public class PathTest {

    private static final double EPS = 1e-6;

    private BezierCurve quarterTurn() {
        return new BezierCurve(
                new Vector2d(0, 0), new Vector2d(1, 0),
                new Vector2d(2, 0), new Vector2d(3, 0),
                0.0, Math.PI / 2);
    }

    private BezierCurve wraparound() {
        return new BezierCurve(
                new Vector2d(0, 0), new Vector2d(1, 0),
                new Vector2d(2, 0), new Vector2d(3, 0),
                Math.toRadians(170), Math.toRadians(-170));
    }


    @Test public void headingAtStartIsStartHeading() {
        assertEquals(0.0, quarterTurn().getHeadingAtT(0), EPS);
    }

    @Test public void headingAtEndIsEndHeading() {
        assertEquals(Math.PI / 2, quarterTurn().getHeadingAtT(1), EPS);
    }

    @Test public void headingAtMidpointIsHalfway() {
        assertEquals(Math.PI / 4, quarterTurn().getHeadingAtT(0.5), EPS);
    }

    @Test public void headingInterpolatesLinearlyWithT() {
        BezierCurve c = quarterTurn();
        assertEquals(Math.PI / 8, c.getHeadingAtT(0.25), EPS);
        assertEquals(3 * Math.PI / 8, c.getHeadingAtT(0.75), EPS);
    }


    @Test public void wraparoundTakesShortestPath() {
        double midHeading = wraparound().getHeadingAtT(0.5);
        assertEquals(Math.PI, Math.abs(midHeading), EPS);
    }

    @Test public void wraparoundEndpointsAreExact() {
        BezierCurve c = wraparound();
        assertEquals(Math.toRadians(170), c.getHeadingAtT(0), EPS);
        assertEquals(Math.toRadians(-170), c.getHeadingAtT(1), EPS);
    }


    @Test public void pathUsesPerCurveHeadingNotGlobalSweep() {
        BezierCurve a = new BezierCurve(
                new Vector2d(0, 0), new Vector2d(1, 0),
                new Vector2d(2, 0), new Vector2d(3, 0),
                0.0, 0.0);
        BezierCurve b = new BezierCurve(
                new Vector2d(3, 0), new Vector2d(4, 0),
                new Vector2d(5, 0), new Vector2d(6, 0),
                Math.PI / 2, Math.PI / 2);
        Path path = new Path(a, b);

        double justBeforeSeam = path.getPointOnPath(2.999).getHeading();
        double justAfterSeam  = path.getPointOnPath(3.001).getHeading();

        assertEquals(0.0, justBeforeSeam, 1e-3);
        assertEquals(Math.PI / 2, justAfterSeam, 1e-3);
    }

    @Test public void matchedSeamHeadingIsContinuous() {
        BezierCurve a = new BezierCurve(
                new Vector2d(0, 0), new Vector2d(1, 0),
                new Vector2d(2, 0), new Vector2d(3, 0),
                0.0, Math.PI / 4);
        BezierCurve b = new BezierCurve(
                new Vector2d(3, 0), new Vector2d(4, 0),
                new Vector2d(5, 0), new Vector2d(6, 0),
                Math.PI / 4, Math.PI / 2);
        Path path = new Path(a, b);

        double atSeam = path.getPointOnPath(3.0).getHeading();
        assertEquals(Math.PI / 4, atSeam, EPS);
    }
}