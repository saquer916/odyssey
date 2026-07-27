package org.firstinspires.ftc.teamcode.odyssey.geometry;

import static org.firstinspires.ftc.teamcode.odyssey.utils.MathUtils.normalizeAngle;

public class Pose2d {
    private final double x;
    private final double y;
    private final double heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = normalizeAngle(heading);
    }

    public Pose2d(Vector2d vector2d, double heading) {
        this.x = vector2d.getX();
        this.y = vector2d.getY();
        this.heading = normalizeAngle(heading);
    }


    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public Vector2d getPosition() {
        Vector2d vector2d = new Vector2d(x, y);
        return vector2d;
    }

    public Vector2d toRobotFrame(Vector2d fieldPoint) {
        fieldPoint = fieldPoint.subtract(new Vector2d(x, y));
        fieldPoint = fieldPoint.rotateVector(-heading);
        return fieldPoint;
    }

    public Vector2d toFieldFrame(Vector2d robotPoint) {
        robotPoint = robotPoint.rotateVector(heading);
        robotPoint = robotPoint.add(new Vector2d(x, y));
        return robotPoint;
    }

    public Pose2d relativeTo(Pose2d other) {
        Vector2d otherVec = other.toRobotFrame(this.getPosition());
        double change = this.heading - other.heading;
        change = normalizeAngle(change);
        return new Pose2d(otherVec, change);
    }

    @Override
    public String toString() {
        return "Pose2d(x=" + x + ", y=" + y + ", heading=" + heading + ")";
    }




}
