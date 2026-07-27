package org.firstinspires.ftc.teamcode.odyssey.path.heading;

import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;

public class ConstantInterpolator implements HeadingInterpolator {
    private final double heading;

    public ConstantInterpolator(double heading) {
        this.heading = heading;
    }
    @Override
    public double getHeading(double t, BezierCurve curve) {
        return heading;
    }
}
