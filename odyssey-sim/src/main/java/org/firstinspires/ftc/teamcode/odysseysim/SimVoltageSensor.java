package org.firstinspires.ftc.teamcode.odysseysim;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Random;

// The hub's battery reading: terminal voltage after sag, with ~5 mV of jitter.
final class SimVoltageSensor implements VoltageSensor {
    private final Simulation sim;
    private final Random random;

    SimVoltageSensor(Simulation sim, Random random) {
        this.sim = sim;
        this.random = random;
    }

    @Override
    public double getVoltage() {
        sim.io(sim.voltageReadMs);
        return sim.physics.batteryVoltage + 0.005 * random.nextGaussian();
    }

    @Override
    public String getDeviceName() {
        return "odyssey-sim hub voltage";
    }
}
