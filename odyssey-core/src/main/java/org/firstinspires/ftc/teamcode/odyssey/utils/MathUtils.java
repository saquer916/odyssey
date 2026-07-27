package org.firstinspires.ftc.teamcode.odyssey.utils;

import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;

public class MathUtils {
    public static final double precisionEPS = 1e-9;
    public static final double abstractEPS = 1e-6;

    private MathUtils() {
        throw new UnsupportedOperationException("Utility Class");
    }

    public static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    public static double min(int first, int ... rest) {
        int min = first;
        for (int num : rest) {
            if (num < min) {
                min = num;
            }
        }
        return min;
    }

    public static double min(double first, double ... rest) {
        double min = first;
        for (double num : rest) {
            if (num < min) {
                min = num;
            }
        }
        return min;
    }

    public static double max(double first, double ... rest) {
        double max = first;
        for (double num : rest) {
            if (num > max) {
                max = num;
            }
        }
        return max;
    }

    public static int max(int first, int ... rest) {
        int max = first;
        for (int num : rest) {
            if (num > max) {
                max = num;
            }
        }
        return max;
    }

    public static double getMagnitudeFromExpression(Vector2d a) {
        return Math.hypot(a.getX(), a.getY());
    }
}
