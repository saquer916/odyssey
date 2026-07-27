package org.firstinspires.ftc.teamcode.odyssey.path.heading;

import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;

public class TangentInterpolator implements HeadingInterpolator {

    public TangentInterpolator() {}
    @Override
    public double getHeading(double t, BezierCurve curve) {
        return curve.getTangentAngle(t);
    }
}
