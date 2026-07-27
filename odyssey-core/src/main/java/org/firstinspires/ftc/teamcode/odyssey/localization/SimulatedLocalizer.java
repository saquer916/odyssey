package org.firstinspires.ftc.teamcode.odyssey.localization;

import org.firstinspires.ftc.teamcode.odyssey.follower.DriveSignal;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;

public class SimulatedLocalizer implements Localizer {

    private Pose2d pose;

    public SimulatedLocalizer(Pose2d startPose) {
        this.pose = startPose;
    }
    @Override
    public Pose2d getPose() {
        return this.pose;
    }

    @Override
    public void update() {

    }

    public Pose2d applyDriveSignal(DriveSignal signal, double deltaTime) {
        double forwardSpeed = signal.getForwardVelocity();
        double strafeSpeed = signal.getStrafeVelocity();
        double turn = signal.getTurn();
        double heading = turn * deltaTime;
        double forwardDistance = forwardSpeed * deltaTime;
        double strafeDistance = strafeSpeed * deltaTime;
        Vector2d forwardStrafe = new Vector2d(forwardDistance, strafeDistance);
        Vector2d fieldFrame = forwardStrafe.rotateVector(pose.getHeading());
        Vector2d poseVec = pose.getPosition();
        Vector2d finalVec = poseVec.add(fieldFrame);
        pose = new Pose2d(finalVec, heading + pose.getHeading());
        return pose;
    }
}
