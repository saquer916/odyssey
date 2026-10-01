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

// What the follower commands when the robot is sitting at the start or end of a path.
// Constants are FirstTest's.
public class FollowerStartEndTest {

    private static final double EPS = 1e-6;

    private static final double MAX_VEL = 1900;
    private static final double MAX_ACCEL = 3000;
    private static final double MAX_BRAKE = 3000;
    private static final double MAX_CENTRIP = 2500;
    private static final double STEP = 1.25;
    private static final double MIN_DRIVE_SPEED = 110;
    private static final double FLOOR_CUTOFF = 500;

    // Straight along +X, evenly spaced control points.
    private Path straight(double length) {
        return new Path(new BezierCurve(
                new Vector2d(0, 0), new Vector2d(length / 3, 0),
                new Vector2d(2 * length / 3, 0), new Vector2d(length, 0), new ConstantInterpolator(0)));
    }

    private VelocityProfile profileOf(Path path) {
        return new VelocityProfile(path, MAX_VEL, MAX_ACCEL, MAX_BRAKE, MAX_CENTRIP, STEP);
    }

    private DriveSignal signalAt(Path path, double x) {
        final Pose2d pose = new Pose2d(x, 0, 0);
        Localizer localizer = new Localizer() {
            @Override public Pose2d getPose() { return pose; }
            @Override public void update() {}
        };
        Follower follower = new Follower(path, localizer, profileOf(path),
                new PIDController(0, 0.25, 0, 0.001), new PIDController(0, 2.0, 0, 0.001),
                MIN_DRIVE_SPEED, FLOOR_CUTOFF);
        return follower.update(0);
    }


    @Test public void profileAccelAtStartIsLaunchAccel() {
        assertEquals(MAX_ACCEL, profileOf(straight(3000)).getTargetTangentialAcceleration(0), EPS);
    }

    @Test public void profileAccelBeforeStartIsZero() {
        assertEquals(0.0, profileOf(straight(3000)).getTargetTangentialAcceleration(-10), EPS);
    }

    @Test public void profileAccelJustBeforeEndIsBraking() {
        Path p = straight(3000);
        assertEquals(-MAX_BRAKE, profileOf(p).getTargetTangentialAcceleration(p.getTotalLength() - 1), EPS);
    }

    @Test public void profileAccelAtEndIsZero() {
        Path p = straight(3000);
        assertEquals(0.0, profileOf(p).getTargetTangentialAcceleration(p.getTotalLength()), EPS);
    }


    @Test public void shortPathStartsMoving() {
        // 100 mm < FLOOR_CUTOFF: the floor never applied, so this was 0 velocity and 0 accel
        DriveSignal signal = signalAt(straight(100), 0);
        assertEquals(MIN_DRIVE_SPEED, signal.getForwardVelocity(), EPS);
        assertEquals(MAX_ACCEL, signal.getForwardAcceleration(), EPS);
    }

    @Test public void longPathStartsMoving() {
        DriveSignal signal = signalAt(straight(3000), 0);
        assertEquals(MIN_DRIVE_SPEED, signal.getForwardVelocity(), EPS);
        assertEquals(MAX_ACCEL, signal.getForwardAcceleration(), EPS);
    }

    @Test public void floorStillReleasesNearTheEnd() {
        // 1 mm from the end the profile is ~77 mm/s; the floor must not hold it at 110
        Path p = straight(3000);
        DriveSignal signal = signalAt(p, p.getTotalLength() - 1);
        assertTrue(signal.getForwardVelocity() < MIN_DRIVE_SPEED);
        assertTrue(signal.getForwardVelocity() > 0);
    }

    @Test public void stoppedAtEndIsNotPushedBack() {
        Path p = straight(100);
        DriveSignal signal = signalAt(p, p.getTotalLength());
        assertEquals(0.0, signal.getForwardVelocity(), EPS);
        assertEquals(0.0, signal.getForwardAcceleration(), EPS);
    }

    @Test public void overshootIsNotPushedBackByBrakeFeedforward() {
        Path p = straight(100);
        DriveSignal signal = signalAt(p, p.getTotalLength() + 20);
        assertEquals(0.0, signal.getForwardAcceleration(), EPS);
        assertTrue("translational PID should pull back", signal.getForwardVelocity() < 0);
    }
}
