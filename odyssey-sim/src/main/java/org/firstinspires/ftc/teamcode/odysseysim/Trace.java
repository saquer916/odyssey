package org.firstinspires.ftc.teamcode.odysseysim;

import java.util.ArrayList;
import java.util.List;

// Fixed-rate recording of the run, in field coordinates (mm, rad).
final class Trace {

    static final class Sample {
        double time;            // seconds since START (negative during INIT)
        String phase;
        double trueX, trueY, trueHeading;
        double odoX, odoY, odoHeading;   // the Pinpoint's estimate, mapped onto the field
        double speed;           // mm/s
        double omega;           // rad/s
        double battery;
        final double[] power = new double[4];   // what the OpMode set on FL, FR, BL, BR
    }

    final List<Sample> samples = new ArrayList<>();

    void record(Simulation sim) {
        DrivePhysics p = sim.physics;
        SimPinpoint pp = sim.pinpoint;
        Sample s = new Sample();
        s.time = sim.timeSinceStart();
        s.phase = sim.phase();
        s.trueX = p.x * 1000.0;
        s.trueY = p.y * 1000.0;
        s.trueHeading = p.heading;
        double[] origin = sim.pinpointOrigin();
        double c = Math.cos(origin[2]);
        double sn = Math.sin(origin[2]);
        s.odoX = origin[0] + c * pp.estX - sn * pp.estY;
        s.odoY = origin[1] + sn * pp.estX + c * pp.estY;
        s.odoHeading = DrivePhysics.normalize(pp.estH + origin[2]);
        s.speed = Math.hypot(p.vx, p.vy) * 1000.0;
        s.omega = p.omega;
        s.battery = p.batteryVoltage;
        for (int i = 0; i < 4; i++) s.power[i] = sim.wheels[i].requestedPower();
        samples.add(s);
    }
}
