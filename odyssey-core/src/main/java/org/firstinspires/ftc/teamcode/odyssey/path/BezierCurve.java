package org.firstinspires.ftc.teamcode.odyssey.path;

import static org.firstinspires.ftc.teamcode.odyssey.utils.MathUtils.normalizeAngle;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.apache.commons.math3.analysis.solvers.BrentSolver;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.path.heading.HeadingInterpolator;

public class BezierCurve {
    private final Vector2d p0;
    private final Vector2d p1;
    private final Vector2d p2;
    private final Vector2d p3;
    private final Vector2d a;
    private final Vector2d b;
    private final Vector2d c;
    private final Vector2d d;

    private HeadingInterpolator headingInterpolator;

    public BezierCurve(Vector2d p0, Vector2d p1, Vector2d p2, Vector2d p3, HeadingInterpolator headingInterpolator) {
        this.p0 = p0;
        this.p1 = p1;
        this.p2 = p2;
        this.p3 = p3;
        this.headingInterpolator = headingInterpolator;
        this.a = p0.scale(-1).add(p1.scale(3)).subtract(p2.scale(3)).add(p3);
        this.b = p0.scale(3).subtract(p1.scale(6)).add(p2.scale(3));
        this.c = p0.scale(-3).add(p1.scale(3));
        this.d = p0;
    }


    public Vector2d getPoint(double t) {
        Vector2d vecp0 = p0.scale((1-t)*(1-t)*(1-t));
        Vector2d vecp1 = p1.scale(3*t*(1-t)*(1-t));
        Vector2d vecp2 = p2.scale(3*(1-t)*t*t);
        Vector2d vecp3 = p3.scale(t*t*t);
        Vector2d sump0p1 = vecp0.add(vecp1);
        Vector2d sump2p3 = vecp2.add(vecp3);
        return sump0p1.add(sump2p3);
    }

    public Vector2d getTangentVector(double t) { // tangent finds heading - derivative of original equation
        Vector2d vecp1p0 = (p1.subtract(p0)).scale(3*(1-t)*(1-t));
        Vector2d vecp2p1 = (p2.subtract(p1)).scale(6*(1-t)*t);
        Vector2d vecp3p2 = (p3.subtract(p2)).scale(3*t*t);
        Vector2d sump1p0p2p1 = vecp2p1.add(vecp1p0);
        return sump1p0p2p1.add(vecp3p2);
    }

    public double getTangentAngle(double t) { // only use if dont need vector and only angle - you can just use getAngleFromCur on the known vector for more efficiency.
        Vector2d tangentVec = getTangentVector(t);
        return tangentVec.getAngleFromCur();
    }

    public double getLength(double b) {
        if (b <= 0) return 0;
        UnivariateFunction speed = t -> getTangentVector(t).getMagnitude();

        IterativeLegendreGaussIntegrator integrator = new IterativeLegendreGaussIntegrator(5, 1e-9, 1e-9);
        return integrator.integrate(1000, speed, 0, b);

    }

    public double getTAtDistance(double dist) {
        if (dist <= 0) return 0;
        if (dist >= getLength(1)) return 1;
        UnivariateFunction brent = b -> getLength(b) - dist;

        BrentSolver solver = new BrentSolver(1e-9, 1e-9);
        return solver.solve(1000, brent, 0, 1);
    }

    public double getHeadingAtT(double t) {
        return headingInterpolator.getHeading(t, this);
    }

    public double getClosestT(Vector2d pose, double coarseStep, double fineStep) {
        double bestT = 0.0;
        double minSqDist = getSquaredDistanceAtT(bestT, pose);
        double distAtOne = getSquaredDistanceAtT(1.0, pose);


        for (double t = 0.0; t <= 1.0; t += coarseStep) {
            double sqDist = getSquaredDistanceAtT(t, pose);
            if (sqDist < minSqDist) {
                minSqDist = sqDist;
                bestT = t;
            }
        }

        if (distAtOne < minSqDist) {
            minSqDist = distAtOne;
            bestT = 1.0;
        }

        double lo = Math.max(0.0, bestT - coarseStep);
        double hi = Math.min(1.0, bestT + coarseStep);

        for (double refinedT = lo; refinedT <= hi; refinedT += fineStep) {
            double sqDist = getSquaredDistanceAtT(refinedT, pose);
            if (sqDist < minSqDist) {
                minSqDist = sqDist;
                bestT = refinedT;
            }
        }

        return bestT;
    }

    public double getClosestT(Vector2d pose) {
        return getClosestT(pose, 0.01, 1e-5);
    }

    public double getSquaredDistanceAtT(double t, Vector2d pose) {

        Vector2d bt = a.scale(t * t * t).add(b.scale(t * t)).add(c.scale(t)).add(d);
        Vector2d delta = bt.subtract(pose);
        return delta.dotProduct(delta);

    }

    public Vector2d getSecondDerivative(double t) {
        return p2.subtract(p1.scale(2)).add(p0).scale(6*(1-t)).add(p3.subtract(p2.scale(2)).add(p1).scale(6*t));

    }
    public double getCurvature(double t) {
        Vector2d first = getTangentVector(t);
        Vector2d second = getSecondDerivative(t);
        return Math.abs(first.crossProduct(second)) / Math.pow(first.getMagnitude(), 3);
    }

    public Vector2d getCentripetalVector(double t) {
        Vector2d v = getTangentVector(t);
        Vector2d e = getSecondDerivative(t);
        return e.subtract(v.scale(e.dotProduct(v)/(v.getMagnitude()*v.getMagnitude())));
    }
}
