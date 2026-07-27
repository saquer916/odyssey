package org.firstinspires.ftc.teamcode.odyssey.follower;

public class DriveSignal {

    private final double forwardVelocity;
    private final double strafeVelocity;
    private final double forwardAcceleration;
    private final double strafeAcceleration;
    private final double turn;

    public DriveSignal(double forwardVelocity, double strafeVelocity, double forwardAcceleration, double strafeAcceleration, double turn) {
        this.forwardVelocity = forwardVelocity;
        this.strafeVelocity = strafeVelocity;
        this.turn = turn;
        this.forwardAcceleration = forwardAcceleration;
        this.strafeAcceleration = strafeAcceleration;
    }

    public double getForwardVelocity() {return forwardVelocity;}
    public double getStrafeVelocity() {return strafeVelocity;}
    public double getTurn() {return turn;}

    public double getForwardAcceleration() {return forwardAcceleration;}
    public double getStrafeAcceleration() {return strafeAcceleration;}
}
