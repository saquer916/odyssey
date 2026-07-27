package org.firstinspires.ftc.teamcode.odyssey.tests;

import static org.junit.Assert.*;

import org.firstinspires.ftc.teamcode.odyssey.control.PIDController;
import org.firstinspires.ftc.teamcode.odyssey.follower.DriveSignal;
import org.firstinspires.ftc.teamcode.odyssey.follower.Follower;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.localization.SimulatedLocalizer;
import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;
import org.firstinspires.ftc.teamcode.odyssey.path.Path;
import org.firstinspires.ftc.teamcode.odyssey.path.VelocityProfile;
import org.junit.Test;

public class FollowerSimTest {

    private static final double DT = 0.02;          // 50Hz
    private static final int    ITERATIONS = 3000;  // 60 seconds of sim

    private Path archPath() {
        return new Path(new BezierCurve(
                new Vector2d(0, 0), new Vector2d(0, 600),
                new Vector2d(1200, 600), new Vector2d(1200, 0),
                Math.toRadians(90), Math.toRadians(-90)));
    }

    private VelocityProfile profileFor(Path p) {
        return new VelocityProfile(p, 1000, 500, 500, 800, 5);
    }

    private PIDController transPid() {
        PIDController pid = new PIDController(0, 1.0, 0, 0);
        pid.setOutputLimits(-1000, 1000);
        return pid;
    }

    private PIDController headingPid() {
        PIDController pid = new PIDController(0, 1.0, 0, 0);
        pid.setOutputLimits(-3, 3);
        return pid;
    }

    /** Runs the sim and returns the final pose. */
    private Pose2d run(Pose2d start, int iterations) {
        Path path = archPath();
        SimulatedLocalizer loc = new SimulatedLocalizer(start);
        Follower follower = new Follower(path, loc, profileFor(path), transPid(), headingPid(), 110, 500);

        double t = 0;
        for (int i = 0; i < iterations; i++) {
            DriveSignal signal = follower.update(t);
            loc.applyDriveSignal(signal, DT);
            t += DT;
        }
        return loc.getPose();
    }

    @Test
    public void reachesTheEndOfThePath() {
        Pose2d end = run(new Pose2d(0, 0.5, Math.toRadians(90)), ITERATIONS);
        assertEquals("x", 1200, end.getX(), 50);
        assertEquals("y", 0, end.getY(), 50);
    }

    @Test
    public void staysNearThePath() {
        Path path = archPath();
        SimulatedLocalizer loc = new SimulatedLocalizer(new Pose2d(0, 0.5, Math.toRadians(90)));
        Follower follower = new Follower(path, loc, profileFor(path), transPid(), headingPid(), 110, 500);

        double t = 0;
        double worstOffset = 0;
        for (int i = 0; i < ITERATIONS; i++) {
            DriveSignal signal = follower.update(t);
            loc.applyDriveSignal(signal, DT);
            t += DT;

            Vector2d pos = loc.getPose().getPosition();
            double d = path.getDistanceOnPath(pos);
            double off = path.getPointOnPath(d).getPosition().subtract(pos).getMagnitude();
            worstOffset = Math.max(worstOffset, off);
        }
        assertTrue("drifted " + worstOffset + "mm off the path", worstOffset < 100);
    }

    @Test
    public void recoversFromABadStart() {
        // dropped 200mm off the path, facing the wrong way
        Pose2d end = run(new Pose2d(-100, -200, 0), ITERATIONS);
        assertEquals("x", 1200, end.getX(), 50);
        assertEquals("y", 0, end.getY(), 50);
    }
}