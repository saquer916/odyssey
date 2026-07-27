package org.firstinspires.ftc.teamcode.odyssey.tests;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;
import org.firstinspires.ftc.teamcode.odyssey.path.Path;
import org.firstinspires.ftc.teamcode.odyssey.path.VelocityProfile;
import org.junit.Test;

public class VelocityProfileTest {

    private static final double EPS = 1e-6;

    
    private static final double MAX_VEL   = 1000;
    private static final double MAX_ACCEL = 500;
    private static final double MAX_BRAKE = 500;
    private static final double MAX_CENTRIP = 800;
    private static final double STEP = 5;

    
    private Path straightPath() {
        return new Path(new BezierCurve(
                new Vector2d(0, 0), new Vector2d(1000, 0),
                new Vector2d(2000, 0), new Vector2d(3000, 0)));
    }

    
    private Path curvyPath() {
        return new Path(new BezierCurve(
                new Vector2d(0, 0), new Vector2d(0, 600),
                new Vector2d(1200, 600), new Vector2d(1200, 0)));
    }

    private VelocityProfile profileOf(Path p) {
        return new VelocityProfile(p, MAX_VEL, MAX_ACCEL, MAX_BRAKE, MAX_CENTRIP, STEP);
    }

    

    @Test public void startsAtRest() {
        assertEquals(0.0, profileOf(straightPath()).getTargetVelocity(0), EPS);
    }

    @Test public void endsAtRest() {
        Path p = straightPath();
        assertEquals(0.0, profileOf(p).getTargetVelocity(p.getTotalLength()), EPS);
    }

    @Test public void clampsBeforeStart() {
        assertEquals(0.0, profileOf(straightPath()).getTargetVelocity(-500), EPS);
    }

    @Test public void clampsPastEnd() {
        Path p = straightPath();
        assertEquals(0.0, profileOf(p).getTargetVelocity(p.getTotalLength() + 5000), EPS);
    }

    

    @Test public void neverExceedsMaxVelocity() {
        Path p = straightPath();
        VelocityProfile vp = profileOf(p);
        for (double d = 0; d <= p.getTotalLength(); d += 10) {
            assertTrue("exceeded max velocity at " + d,
                    vp.getTargetVelocity(d) <= MAX_VEL + EPS);
        }
    }

    @Test public void longStraightReachesCruise() {
        
        Path p = straightPath();
        assertEquals(MAX_VEL, profileOf(p).getTargetVelocity(p.getTotalLength() / 2), 1.0);
    }

    

    @Test public void accelerationIsRespectedFromStart() {
        
        VelocityProfile vp = profileOf(straightPath());
        double expected = Math.sqrt(2 * MAX_ACCEL * 100);
        assertEquals(expected, vp.getTargetVelocity(100), 5.0);
    }

    @Test public void brakingIsRespectedIntoEnd() {
        
        Path p = straightPath();
        VelocityProfile vp = profileOf(p);
        double expected = Math.sqrt(2 * MAX_BRAKE * 100);
        assertEquals(expected, vp.getTargetVelocity(p.getTotalLength() - 100), 5.0);
    }

    @Test public void speedIsAchievableEverywhere() {
        
        Path p = straightPath();
        VelocityProfile vp = profileOf(p);
        double ds = 1.0;
        for (double d = 0; d + ds <= p.getTotalLength(); d += ds) {
            double v0 = vp.getTargetVelocity(d);
            double v1 = vp.getTargetVelocity(d + ds);
            double maxReachable = Math.sqrt(v0 * v0 + 2 * MAX_ACCEL * ds);
            double minReachable = Math.sqrt(Math.max(0, v0 * v0 - 2 * MAX_BRAKE * ds));
            assertTrue("accelerates too hard at " + d, v1 <= maxReachable + 1e-3);
            assertTrue("brakes too hard at " + d,      v1 >= minReachable - 1e-3);
        }
    }

    

    @Test public void slowsForTheCurve() {
        
        
        Path curvy = curvyPath();
        VelocityProfile vp = profileOf(curvy);
        double mid = curvy.getTotalLength() / 2;
        double vCurve = vp.getTargetVelocity(mid);

        assertTrue("curve did not limit speed", vCurve < MAX_VEL);
    }

    @Test public void curveLimitMatchesPhysics() {
        
        Path curvy = curvyPath();
        VelocityProfile vp = profileOf(curvy);
        double mid = curvy.getTotalLength() / 2;

        double kappa = curvy.getCurvatureFromPathDistance(mid);
        double physicsLimit = Math.sqrt(MAX_CENTRIP / kappa);

        
        assertTrue(vp.getTargetVelocity(mid) <= physicsLimit + 1.0);
    }

    @Test public void straightPathHasNoCurveLimit() {
        
        
        Path p = straightPath();
        assertEquals(MAX_VEL, profileOf(p).getTargetVelocity(p.getTotalLength() / 2), 1.0);
    }

    

    @Test public void interpolatesBetweenSamples() {
        
        VelocityProfile vp = profileOf(straightPath());
        double a = vp.getTargetVelocity(100);
        double b = vp.getTargetVelocity(105);
        double mid = vp.getTargetVelocity(102.5);
        assertTrue(mid >= Math.min(a, b) - EPS);
        assertTrue(mid <= Math.max(a, b) + EPS);
    }

    @Test public void isMonotonicIncreasingAtStart() {
        
        VelocityProfile vp = profileOf(straightPath());
        double prev = vp.getTargetVelocity(0);
        for (double d = 5; d <= 500; d += 5) {
            double v = vp.getTargetVelocity(d);
            assertTrue("speed dropped while accelerating at " + d, v >= prev - EPS);
            prev = v;
        }
    }
}