package org.firstinspires.ftc.teamcode.odyssey.follower;

import static org.firstinspires.ftc.teamcode.odyssey.utils.MathUtils.normalizeAngle;

import org.firstinspires.ftc.teamcode.odyssey.control.PIDController;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.localization.Localizer;
import org.firstinspires.ftc.teamcode.odyssey.path.Path;
import org.firstinspires.ftc.teamcode.odyssey.path.VelocityProfile;

public class Follower {

    private final Path path;
    private final Localizer localizer;
    private final VelocityProfile velocityProfile;
    private final PIDController pidTranslational;
    private final PIDController pidHeading;

    private final double totalLength;
    private final double minDriveSpeed;
    private final double floorCutoff;
    private double lastDistanceRemaining;

    public Follower(Path path, Localizer localizer, VelocityProfile velocityProfile, PIDController pidTranslational, PIDController pidHeading, double minDriveSpeed, double floorCutoff) {
        this.path = path;
        this.localizer = localizer;
        this.velocityProfile = velocityProfile;
        this.pidTranslational = pidTranslational;
        this.pidHeading = pidHeading;
        this.totalLength = path.getTotalLength();
        this.minDriveSpeed = minDriveSpeed;
        this.floorCutoff = floorCutoff;
    }

    public DriveSignal update(double currentTime) {
        Pose2d pose = localizer.getPose();
        double distance = path.getDistanceOnPath(pose.getPosition());
        Pose2d reference = path.getPointOnPath(distance);
        double distanceRemaining = totalLength - distance;
        this.lastDistanceRemaining = distanceRemaining;
        double profileSpeed = velocityProfile.getTargetVelocity(distance);
        double cmdSpeed = (distanceRemaining > floorCutoff) ? Math.max(profileSpeed, minDriveSpeed) : profileSpeed;

        // translational vector
        Vector2d offset = reference.getPosition().subtract(pose.getPosition());
        double pidOutputTranslational = pidTranslational.getOutput(currentTime, -offset.getMagnitude());
        Vector2d offsetNormalized = offset.normalize();
        Vector2d translationalVector = offsetNormalized.scale(pidOutputTranslational);

        // drive vector
        Vector2d tangent = path.getTangentFromPathDistance(distance);
        Vector2d driveVectorNormalize = tangent.normalize();
        Vector2d driveVector = driveVectorNormalize.scale(cmdSpeed);

        // heading pid
        double referenceHeading = reference.getHeading();
        double poseHeading = pose.getHeading();
        double diff = normalizeAngle(referenceHeading - poseHeading);
        double heading = pidHeading.getOutput(currentTime, -diff);

        // turning into one
        Vector2d sum = translationalVector.add(driveVector);
        Vector2d velocityVector = sum.rotateVector(-pose.getHeading());


        // accel


        // centripetal vector
        Vector2d centripetal = path.getCentripetalVectorPath(distance);
        double curvature = path.getCurvatureFromPathDistance(distance);
        Vector2d centripetalVectorNormalize = centripetal.normalize();
        Vector2d centripetalVector = centripetalVectorNormalize.scale(cmdSpeed * cmdSpeed * curvature);

        // tangential vector
        double profileAccel = velocityProfile.getTargetTangentialAcceleration(distance);
        Vector2d tangentialVector = driveVectorNormalize.scale(profileAccel);

        // turning it into one
        Vector2d accelSum = centripetalVector.add(tangentialVector);
        Vector2d accelVector = accelSum.rotateVector(-pose.getHeading());

        // applying to drive signal
        return new DriveSignal(velocityVector.getX(), velocityVector.getY(), accelVector.getX(), accelVector.getY(), heading);
    }

    // Valid only after at least one update() call — reuses that call's distance query
    // instead of making the caller run a second heavy getDistanceOnPath() pass.
    public double getDistanceRemaining() {
        return lastDistanceRemaining;
    }

}
