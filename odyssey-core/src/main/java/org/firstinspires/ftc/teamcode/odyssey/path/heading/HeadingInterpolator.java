package org.firstinspires.ftc.teamcode.odyssey.path.heading;

import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;

public interface HeadingInterpolator {
    double getHeading(double t, BezierCurve curve);
}
