package org.firstinspires.ftc.robotcore.external.navigation;

// odyssey-sim stand-in. Like the SDK's AngleUnit, results are normalized to [-180, 180) / [-pi, pi).
public enum AngleUnit {
    DEGREES, RADIANS;

    public double fromDegrees(double degrees) {
        return this == RADIANS ? normalize(Math.toRadians(degrees)) : normalize(degrees);
    }

    public double fromRadians(double radians) {
        return this == RADIANS ? normalize(radians) : normalize(Math.toDegrees(radians));
    }

    public double fromUnit(AngleUnit them, double theirs) {
        return them == DEGREES ? fromDegrees(theirs) : fromRadians(theirs);
    }

    public double toDegrees(double angle) {
        return this == RADIANS ? normalizeDegrees(Math.toDegrees(angle)) : normalizeDegrees(angle);
    }

    public double toRadians(double angle) {
        return this == DEGREES ? normalizeRadians(Math.toRadians(angle)) : normalizeRadians(angle);
    }

    public double normalize(double angle) {
        return this == RADIANS ? normalizeRadians(angle) : normalizeDegrees(angle);
    }

    public static double normalizeDegrees(double degrees) {
        while (degrees >= 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    public static double normalizeRadians(double radians) {
        while (radians >= Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }

    public UnnormalizedAngleUnit getUnnormalized() {
        return this == DEGREES ? UnnormalizedAngleUnit.DEGREES : UnnormalizedAngleUnit.RADIANS;
    }
}
