package org.firstinspires.ftc.robotcore.external.navigation;

// odyssey-sim stand-in.
public class Pose2D {
    protected final double x;
    protected final double y;
    protected final DistanceUnit distanceUnit;
    protected final double heading;
    protected final AngleUnit headingUnit;

    public Pose2D(DistanceUnit distanceUnit, double x, double y, AngleUnit headingUnit, double heading) {
        this.x = x;
        this.y = y;
        this.distanceUnit = distanceUnit;
        this.heading = heading;
        this.headingUnit = headingUnit;
    }

    public double getX(DistanceUnit unit) { return unit.fromUnit(distanceUnit, x); }
    public double getY(DistanceUnit unit) { return unit.fromUnit(distanceUnit, y); }
    public double getHeading(AngleUnit unit) { return unit.fromUnit(headingUnit, heading); }

    @Override
    public String toString() {
        return String.format("Pose2D(x=%.2f mm, y=%.2f mm, heading=%.2f deg)",
                getX(DistanceUnit.MM), getY(DistanceUnit.MM), getHeading(AngleUnit.DEGREES));
    }
}
