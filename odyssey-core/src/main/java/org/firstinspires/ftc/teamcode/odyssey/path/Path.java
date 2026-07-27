package org.firstinspires.ftc.teamcode.odyssey.path;

import org.firstinspires.ftc.teamcode.odyssey.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;

public class Path {
    private final BezierCurve[] curves;
    private final double[] curveLengths; // cached curve.getLength(1) per curve — computed once, not per query
    private final double totalLength;

    public Path(BezierCurve... curves) { // ellipsis means varargs, can pass any amount of BezierCurves and it will auto store into curves array
        this.curves = curves;
        this.curveLengths = new double[curves.length];
        double sum = 0;
        for (int i = 0; i < curves.length; i++) {
            curveLengths[i] = curves[i].getLength(1);
            sum += curveLengths[i];
        }
        this.totalLength = sum;
    }

    public double getTotalLength() {
        return totalLength;
    }

    public Pose2d getPointOnPath(double distance) {
        double dist = distance;
        if (dist <= 0) return new Pose2d(curves[0].getPoint(0), curves[0].getHeadingAtT(0));
        for (int i = 0; i < curves.length; i++) {
            BezierCurve curve = curves[i];
            double len = curveLengths[i];
            if (dist <= len) {
                double t = curve.getTAtDistance(dist);
                double heading = curve.getHeadingAtT(t);
                return new Pose2d(curve.getPoint(t), heading);
            }
            else {
                dist -= len;
            }
        }
        return new Pose2d(curves[curves.length - 1].getPoint(1), curves[curves.length - 1].getHeadingAtT(1));
    }

    public double getDistanceOnPath(Vector2d pose) {
        double cumilativeLength = 0;
        double bestDist = Double.MAX_VALUE;
        double pathDistance = 0;
        for (int i = 0; i < curves.length; i++) {
            BezierCurve curve = curves[i];
            double t = curve.getClosestT(pose);
            double dist = curve.getSquaredDistanceAtT(t, pose);
            if (dist < bestDist) {
                bestDist = dist;
                pathDistance = cumilativeLength + curve.getLength(t);

            }

            cumilativeLength += curveLengths[i];

        }
        return pathDistance;
    }

    public double getCurvatureFromPathDistance(double dist) {
        if (dist <= 0) return curves[0].getCurvature(0);
        for (int i = 0; i < curves.length; i++) {
            BezierCurve curve = curves[i];
            double len = curveLengths[i];
            if (dist <= len) {
                double t = curve.getTAtDistance(dist);
                return curve.getCurvature(t);
            }
            else {
                dist -= len;
            }
        }
        return curves[curves.length - 1].getCurvature(1);
    }

    public Vector2d getTangentFromPathDistance(double dist) {
        if (dist <= 0) return curves[0].getTangentVector(0);
        for (int i = 0; i < curves.length; i++) {
            BezierCurve curve = curves[i];
            double len = curveLengths[i];
            if (dist <= len) {
                double t = curve.getTAtDistance(dist);
                return curve.getTangentVector(t);
            }
            else {
                dist -= len;
            }
        }
        return curves[curves.length - 1].getTangentVector(1);
    }

    public Vector2d getCentripetalVectorPath(double dist) {
        if (dist <= 0) return curves[0].getCentripetalVector(0);
        for (int i = 0; i < curves.length; i++) {
            BezierCurve curve = curves[i];
            double len = curveLengths[i];
            if (dist <= len) {
                double t = curve.getTAtDistance(dist);
                return curve.getCentripetalVector(t);
            }
            else {
                dist -= len;
            }
        }
        return curves[curves.length - 1].getCentripetalVector(1);
    }



}
