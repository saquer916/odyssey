package org.firstinspires.ftc.robotcore.external.navigation;

// odyssey-sim stand-in.
public enum UnnormalizedAngleUnit {
    DEGREES, RADIANS;

    public double fromDegrees(double degrees) {
        return this == RADIANS ? Math.toRadians(degrees) : degrees;
    }

    public double fromRadians(double radians) {
        return this == RADIANS ? radians : Math.toDegrees(radians);
    }

    public double toDegrees(double angle) {
        return this == RADIANS ? Math.toDegrees(angle) : angle;
    }

    public double toRadians(double angle) {
        return this == DEGREES ? Math.toRadians(angle) : angle;
    }

    public AngleUnit getNormalized() {
        return this == DEGREES ? AngleUnit.DEGREES : AngleUnit.RADIANS;
    }
}
