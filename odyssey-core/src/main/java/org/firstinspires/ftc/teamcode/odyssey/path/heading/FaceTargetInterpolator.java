package org.firstinspires.ftc.teamcode.odyssey.path.heading;

import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;

public class FaceTargetInterpolator implements HeadingInterpolator {
    private final Vector2d target;

    public FaceTargetInterpolator(Vector2d target) {
        this.target = target;
    }

    @Override
    public double getHeading(double t, BezierCurve curve) {
        Vector2d p = curve.getPoint(t);
        return Math.atan2(target.getY() - p.getY(), target.getX() - p.getX());
    }
}
