package org.firstinspires.ftc.teamcode.odysseysim;

import java.util.Random;

// What the Pinpoint computes internally (~1.5 kHz, so continuously here), in its own frame, mm / rad.
// "ideal" is the robot's true pose in that frame; "est" is what the device believes (ideal plus
// pod noise and heading drift). resetPosAndIMU / setPos* move both, so the frame the OpMode works
// in can always be mapped back onto the field.
final class SimPinpoint {
    private final Random random;
    private final double noisePerSqrtMm;
    private final double headingDriftPerSecond;
    private final double readNoiseMm;
    private final double readNoiseRad;
    private final double velocityNoise;

    double estX, estY, estH;
    double idealX, idealY, idealH;
    private double velX, velY, velH;

    // Snapshot from the last update(), which is what the driver's getters return.
    double readX, readY, readH, readVelX, readVelY, readVelH;

    SimPinpoint(SimConfig c, Random random) {
        this.random = random;
        noisePerSqrtMm = c.number("pinpoint.noiseMmPerSqrtM") / Math.sqrt(1000.0);
        headingDriftPerSecond = Math.toRadians(c.number("pinpoint.headingDriftDegPerMin")) / 60.0;
        readNoiseMm = c.number("pinpoint.readNoiseMm");
        readNoiseRad = Math.toRadians(c.number("pinpoint.readNoiseDeg"));
        velocityNoise = c.number("pinpoint.velocityNoiseMmPerSec");
    }

    // vx, vy in m/s (robot frame), omega in rad/s.
    void onPhysicsStep(double dt, double vx, double vy, double omega) {
        double dx = vx * dt * 1000.0;
        double dy = vy * dt * 1000.0;
        double dh = omega * dt;

        double gyro = dh + headingDriftPerSecond * dt;
        double noisyDx = dx + noisePerSqrtMm * Math.sqrt(Math.abs(dx)) * random.nextGaussian();
        double noisyDy = dy + noisePerSqrtMm * Math.sqrt(Math.abs(dy)) * random.nextGaussian();
        double mid = estH + gyro / 2;
        double c = Math.cos(mid);
        double s = Math.sin(mid);
        velX = (dx * c - dy * s) / dt;
        velY = (dx * s + dy * c) / dt;
        velH = gyro / dt;
        estX += noisyDx * c - noisyDy * s;
        estY += noisyDx * s + noisyDy * c;
        estH += gyro;

        double idealMid = idealH + dh / 2;
        idealX += dx * Math.cos(idealMid) - dy * Math.sin(idealMid);
        idealY += dx * Math.sin(idealMid) + dy * Math.cos(idealMid);
        idealH += dh;
    }

    void snapshot() {
        readX = estX + readNoiseMm * random.nextGaussian();
        readY = estY + readNoiseMm * random.nextGaussian();
        readH = estH + readNoiseRad * random.nextGaussian();
        readVelX = velX + velocityNoise * random.nextGaussian();
        readVelY = velY + velocityNoise * random.nextGaussian();
        readVelH = velH;
    }

    void reset() {
        estX = estY = estH = 0;
        idealX = idealY = idealH = 0;
        velX = velY = velH = 0;
    }

    void setX(double mm) { estX = mm; idealX = mm; }
    void setY(double mm) { estY = mm; idealY = mm; }
    void setHeading(double rad) { estH = rad; idealH = rad; }
}
