package org.firstinspires.ftc.teamcode.odysseysim;

// Rigid-body mecanum drivetrain, SI units (m, s, N, rad).
//
// Each wheel is a DC motor at the wheel surface: force = stallForce * (V / 12 - u / freeSpeed),
// V being the PWM'd battery voltage and u the wheel's surface speed. At zero power with BRAKE the
// motor leads are shorted (V = 0), so back-EMF brakes the wheel; FLOAT leaves them open.
// Friction is Coulomb (constant, against u) with a small deadband acting as static friction.
//
// Wheels connect to the body through the same mixing MecanumDrive uses:
//   u_i = vx * FWD_i + vy * STRAFE_i + omega * k * TURN_i,   k = lX + lY
// and by virtual work a wheel force f_i pushes the body with f_i * (FWD_i, STRAFE_i, k * TURN_i).
// Back-EMF and the friction deadband are solved implicitly so the step stays stable.
final class DrivePhysics {
    static final double NOMINAL_VOLTAGE = 12.0;
    // Wheel order everywhere in the simulator: FL, FR, BL, BR.
    static final double[] FWD = {1, 1, 1, 1};
    static final double[] STRAFE = {-1, 1, 1, -1};
    static final double[] TURN = {-1, 1, -1, 1};

    final double mass;
    final double inertia;
    final double k;
    final double stallForce;
    final double freeSpeed;
    final double stallCurrent;
    final double friction;
    final double frictionDeadband;
    final double strafeEfficiency;
    final double openCircuitVoltage;
    final double internalResistance;
    final double halfLength;
    final double halfWidth;
    final double fieldHalf;
    final boolean walls;

    // Field pose of the robot center, and robot-frame velocity: +x forward, +y left, +omega CCW.
    double x, y, heading;
    double vx, vy, omega;
    double batteryVoltage;
    final double[] wheelTravel = new double[4];
    double firstWallHit = Double.NaN;

    DrivePhysics(SimConfig c) {
        mass = c.number("robot.massKg");
        inertia = c.number("robot.inertiaKgM2");
        k = (c.number("robot.lXmm") + c.number("robot.lYmm")) / 1000.0;
        stallForce = c.number("motor.stallForceN");
        freeSpeed = c.number("motor.freeSpeedMmPerSec") / 1000.0;
        stallCurrent = c.number("motor.stallCurrentA");
        friction = c.number("motor.frictionN");
        frictionDeadband = c.number("motor.frictionDeadbandMmPerSec") / 1000.0;
        strafeEfficiency = c.number("drive.strafeEfficiency");
        openCircuitVoltage = c.number("battery.openCircuitVolts");
        internalResistance = c.number("battery.internalOhms");
        halfLength = c.number("robot.lengthMm") / 2000.0;
        halfWidth = c.number("robot.widthMm") / 2000.0;
        fieldHalf = c.number("field.sizeMm") / 2000.0;
        walls = c.flag("field.walls");
        x = c.number("start.xMm") / 1000.0;
        y = c.number("start.yMm") / 1000.0;
        heading = Math.toRadians(c.number("start.headingDeg"));
        batteryVoltage = openCircuitVoltage;
    }

    double wheelSpeed(int i) {
        return FWD[i] * vx + STRAFE[i] * vy + TURN[i] * k * omega;
    }

    // power[i]: fraction of battery voltage on wheel i, signed in the wheel's forward sense.
    // braking[i]: leads shorted at zero power (BRAKE) rather than open (FLOAT).
    void step(double dt, double[] power, boolean[] braking, double time) {
        double[][] m = {{mass, 0, 0}, {0, mass, 0}, {0, 0, inertia}};
        double[] rhs = {mass * vx, mass * vy, inertia * omega};
        double drawn = 0;

        for (int i = 0; i < 4; i++) {
            double[] a = {FWD[i], STRAFE[i], TURN[i] * k};
            double u = wheelSpeed(i);
            double q = power[i];
            double drive = 0;
            double damping = 0;
            double coulomb = 0;
            if (q != 0 || braking[i]) {
                double voltageFraction = q * batteryVoltage / NOMINAL_VOLTAGE;
                drive = stallForce * voltageFraction;
                damping += stallForce / freeSpeed;
                drawn += q * stallCurrent * (voltageFraction - u / freeSpeed);
            }
            if (Math.abs(u) < frictionDeadband) {
                damping += friction / frictionDeadband;
            } else {
                coulomb = -friction * Math.signum(u);
            }
            for (int r = 0; r < 3; r++) {
                double driveShare = r == 1 ? strafeEfficiency : 1.0;
                rhs[r] += dt * (drive * driveShare + coulomb) * a[r];
                for (int col = 0; col < 3; col++) m[r][col] += dt * damping * a[r] * a[col];
            }
        }

        double[] v = solve3(m, rhs);
        vx = v[0];
        vy = v[1];
        omega = v[2];
        batteryVoltage = openCircuitVoltage - internalResistance * Math.max(0, drawn);

        double midHeading = heading + 0.5 * omega * dt;
        x += (vx * Math.cos(midHeading) - vy * Math.sin(midHeading)) * dt;
        y += (vx * Math.sin(midHeading) + vy * Math.cos(midHeading)) * dt;
        heading = normalize(heading + omega * dt);
        for (int i = 0; i < 4; i++) wheelTravel[i] += wheelSpeed(i) * dt;

        if (walls) collideWithWalls(time);
    }

    private void collideWithWalls(double time) {
        double c = Math.abs(Math.cos(heading));
        double s = Math.abs(Math.sin(heading));
        double extentX = halfLength * c + halfWidth * s;
        double extentY = halfLength * s + halfWidth * c;
        double fieldVx = vx * Math.cos(heading) - vy * Math.sin(heading);
        double fieldVy = vx * Math.sin(heading) + vy * Math.cos(heading);
        boolean hit = false;
        if (x > fieldHalf - extentX) { x = fieldHalf - extentX; fieldVx = Math.min(0, fieldVx); hit = true; }
        if (x < -fieldHalf + extentX) { x = -fieldHalf + extentX; fieldVx = Math.max(0, fieldVx); hit = true; }
        if (y > fieldHalf - extentY) { y = fieldHalf - extentY; fieldVy = Math.min(0, fieldVy); hit = true; }
        if (y < -fieldHalf + extentY) { y = -fieldHalf + extentY; fieldVy = Math.max(0, fieldVy); hit = true; }
        if (hit) {
            vx = fieldVx * Math.cos(heading) + fieldVy * Math.sin(heading);
            vy = -fieldVx * Math.sin(heading) + fieldVy * Math.cos(heading);
            if (Double.isNaN(firstWallHit)) firstWallHit = time;
        }
    }

    // What an ideal feedforward fit would find for this robot driving forward, at 12 V.
    double equivalentKs() { return friction / stallForce; }
    double equivalentKv() { return 1.0 / (freeSpeed * 1000.0); }
    double equivalentKa() { return mass / (4.0 * stallForce) / 1000.0; }

    static double normalize(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    // Gaussian elimination with partial pivoting.
    static double[] solve3(double[][] m, double[] b) {
        double[][] a = new double[3][4];
        for (int r = 0; r < 3; r++) {
            System.arraycopy(m[r], 0, a[r], 0, 3);
            a[r][3] = b[r];
        }
        for (int col = 0; col < 3; col++) {
            int pivot = col;
            for (int r = col + 1; r < 3; r++) if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) pivot = r;
            double[] tmp = a[col];
            a[col] = a[pivot];
            a[pivot] = tmp;
            for (int r = col + 1; r < 3; r++) {
                double f = a[r][col] / a[col][col];
                for (int c = col; c < 4; c++) a[r][c] -= f * a[col][c];
            }
        }
        double[] x = new double[3];
        for (int r = 2; r >= 0; r--) {
            double sum = a[r][3];
            for (int c = r + 1; c < 3; c++) sum -= a[r][c] * x[c];
            x[r] = sum / a[r][r];
        }
        return x;
    }
}
