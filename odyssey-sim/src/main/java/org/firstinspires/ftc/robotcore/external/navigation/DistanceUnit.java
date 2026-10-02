package org.firstinspires.ftc.robotcore.external.navigation;

// odyssey-sim stand-in.
public enum DistanceUnit {
    METER(1000.0), CM(10.0), MM(1.0), INCH(25.4);

    private final double mmPerUnit;

    DistanceUnit(double mmPerUnit) {
        this.mmPerUnit = mmPerUnit;
    }

    public double fromMm(double mm) { return mm / mmPerUnit; }
    public double toMm(double value) { return value * mmPerUnit; }
    public double fromCm(double cm) { return fromMm(cm * 10.0); }
    public double toCm(double value) { return toMm(value) / 10.0; }
    public double fromMeters(double meters) { return fromMm(meters * 1000.0); }
    public double toMeters(double value) { return toMm(value) / 1000.0; }
    public double fromInches(double inches) { return fromMm(inches * 25.4); }
    public double toInches(double value) { return toMm(value) / 25.4; }
    public double fromUnit(DistanceUnit unit, double value) { return fromMm(unit.toMm(value)); }
}
