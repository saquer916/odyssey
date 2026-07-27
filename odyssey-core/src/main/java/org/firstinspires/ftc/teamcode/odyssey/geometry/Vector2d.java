package org.firstinspires.ftc.teamcode.odyssey.geometry;

public class Vector2d {
    private final double x;
    private final double y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getMagnitude() {
        return Math.hypot(x, y);
    }
    public Vector2d rotateVector(double theta) {
        double outputX = (x*Math.cos(theta)) - (y*Math.sin(theta));
        double outputY = (x*Math.sin(theta)) + (y*Math.cos(theta));
        return new Vector2d(outputX, outputY);
    }

    public Vector2d subtract(Vector2d other) {
        return new Vector2d(x - other.getX(), y - other.getY());
    }

    public Vector2d add(Vector2d other) {
        return new Vector2d(x + other.getX(), y + other.getY());
    }
    public Vector2d scale(double factor) {
        return new Vector2d(x*factor, y*factor);
    }

    public double dotProduct(Vector2d other) {
        return (x * other.getX()) + (y * other.getY());
    }

    public double getAngleFromCur() {
        return Math.atan2(this.y, this.x);
    }

    public double crossProduct(Vector2d other) {
        return (x * other.getY()) - (y * other.getX());
    }

    public Vector2d normalize() {
        double mag = this.getMagnitude();
        if (mag == 0.0) {
            return new Vector2d(0.0, 0.0);
        }

        return new Vector2d(x / mag, y / mag);
    }

}
