package org.firstinspires.ftc.teamcode.odyssey.tests;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;
import org.junit.Test;

public class ClosestTValueTest {

    
    private static final double EPS = 1e-4;

    
    private BezierCurve arch() {
        return new BezierCurve(
                new Vector2d(0, 0), new Vector2d(0, 3),
                new Vector2d(6, 3), new Vector2d(6, 0));
    }

    
    private BezierCurve straightLine() {
        return new BezierCurve(
                new Vector2d(0, 0), new Vector2d(1, 0),
                new Vector2d(2, 0), new Vector2d(3, 0));
    }

    

    @Test public void robotAtStartGivesTZero() {
        
        assertEquals(0.0, arch().getClosestT(new Vector2d(0, 0)), EPS);
    }

    @Test public void robotAtEndGivesTOne() {
        
        assertEquals(1.0, arch().getClosestT(new Vector2d(6, 0)), EPS);
    }

    @Test public void robotFarPastEndClampsToOne() {
        
        assertEquals(1.0, arch().getClosestT(new Vector2d(100, 0)), EPS);
    }

    @Test public void robotFarBeforeStartClampsToZero() {
        
        assertEquals(0.0, arch().getClosestT(new Vector2d(-100, 0)), EPS);
    }



    @Test
    public void peakGivesTHalf() {
        assertEquals(0.5, arch().getClosestT(new Vector2d(3, 10)), 1e-4);
    }

    

    @Test public void straightLineMidpoint() {
        

        assertEquals(0.5, straightLine().getClosestT(new Vector2d(1.5, 5)), EPS);
    }

    @Test public void straightLineQuarter() {
        
        assertEquals(0.25, straightLine().getClosestT(new Vector2d(0.75, 5)), EPS);
    }

    

    @Test public void resultIsNoFartherThanNeighbors() {
        
        
        BezierCurve c = arch();
        Vector2d robot = new Vector2d(1.5, 4);   
        double t = c.getClosestT(robot);

        double dHere = c.getPoint(t).subtract(robot).getMagnitude();
        double dLeft  = c.getPoint(Math.max(0, t - 0.02)).subtract(robot).getMagnitude();
        double dRight = c.getPoint(Math.min(1, t + 0.02)).subtract(robot).getMagnitude();

        assertTrue(dHere <= dLeft + 1e-9);
        assertTrue(dHere <= dRight + 1e-9);
    }
}