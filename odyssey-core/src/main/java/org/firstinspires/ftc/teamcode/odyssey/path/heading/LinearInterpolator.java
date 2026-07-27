package org.firstinspires.ftc.teamcode.odyssey.path.heading;

import static org.firstinspires.ftc.teamcode.odyssey.utils.MathUtils.normalizeAngle;

import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;

public class LinearInterpolator implements HeadingInterpolator {
    private final double startHeading;
    private final double endHeading;

    public LinearInterpolator(double startHeading, double endHeading) {
        this.startHeading = startHeading;
        this.endHeading = endHeading;
    }
    @Override
    public double getHeading(double t, BezierCurve curve) {
        double delta = normalizeAngle(endHeading - startHeading);
        return normalizeAngle(startHeading + delta * t);
    }
}
