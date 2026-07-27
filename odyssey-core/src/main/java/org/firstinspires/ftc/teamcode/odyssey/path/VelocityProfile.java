package org.firstinspires.ftc.teamcode.odyssey.path;

import static org.firstinspires.ftc.teamcode.odyssey.utils.MathUtils.min;

public class VelocityProfile {

    private double[] velocities;
    private double[] distances;
    private final double step;
    public VelocityProfile(Path path, double maxVelocity, double maxAcceleration, double maxBrake, double maxCentripetalAccel, double step) {

        this.step = step;

        double totalDistance = path.getTotalLength();

        int arraySize = (int) Math.ceil(totalDistance / step) + 1;
        distances = new double[arraySize];
        velocities = new double[arraySize];

        for (int i = 0; i < arraySize; i++) {
            distances[i] = i * step;

            if (distances[i] > totalDistance) {
                distances[i] = totalDistance;
            }

            double curvature = path.getCurvatureFromPathDistance(distances[i]);

            double curveLimit = Math.sqrt(maxCentripetalAccel / curvature);
            if (curvature < 1e-4) curveLimit = maxVelocity;

            velocities[i] = min(maxVelocity, curveLimit);
        }

        velocities[0] = 0;
        for (int i = 1; i < arraySize; i++) {
            double ds = distances[i] - distances[i-1];
            velocities[i] = min(velocities[i], Math.sqrt((velocities[i-1] * velocities[i-1]) + 2 * maxAcceleration * ds));
        }

        velocities[arraySize - 1] = 0;
        for (int i = arraySize - 2; i >= 0; i--) {
            double ds = distances[i+1] - distances[i];
            velocities[i] = min(velocities[i], Math.sqrt((velocities[i+1] * velocities[i+1]) + 2 * maxBrake * ds));
        }
    }
    public double getTargetVelocity(double distance) {
        if (distances == null || distances.length == 0) return 0;

        int length = distances.length;

        if (distance <= 0) return velocities[0];
        if (distance >= distances[length - 1]) return velocities[length - 1];

        int lowIdx = (int) Math.floor(distance / step);

        if (lowIdx >= length - 1) lowIdx = length - 2;
        int highIdx = lowIdx + 1;

        double d1 = distances[lowIdx];
        double d2 = distances[highIdx];
        double v1 = velocities[lowIdx];
        double v2 = velocities[highIdx];

        double res = v1*v1 + (distance - d1) * ((v2*v2 - v1*v1) / (d2 - d1));

        return Math.sqrt(res);
    }

    public double getTargetTangentialAcceleration(double distance) {
        if (distances == null || distances.length == 0) return 0;

        int length = distances.length;
        if (distance <= 0) return 0;
        if (distance > distances[length - 1]) return 0;

        int lowIdx = (int) Math.floor(distance / step);

        if (lowIdx >= length - 1) lowIdx = length - 2;
        int highIdx = lowIdx + 1;

        double d1 = distances[lowIdx];
        double d2 = distances[highIdx];
        double v0 = velocities[lowIdx];
        double vf = velocities[highIdx];

        double ds = d2 - d1;
        return (vf*vf - v0*v0) / (2*ds);
    }
}
