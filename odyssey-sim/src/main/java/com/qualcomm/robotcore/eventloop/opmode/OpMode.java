package com.qualcomm.robotcore.eventloop.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.odysseysim.SimClock;

// odyssey-sim stand-in for the FTC SDK class. The simulator sets hardwareMap / telemetry before
// init() and calls init, init_loop, start, loop and stop like the robot controller does.
public abstract class OpMode {
    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Gamepad gamepad1 = new Gamepad();
    public Gamepad gamepad2 = new Gamepad();
    public double time;

    private long startNanos = SimClock.nanoTime();
    private boolean stopRequested;

    public abstract void init();

    public void init_loop() {}

    public void start() {}

    public abstract void loop();

    public void stop() {}

    public double getRuntime() {
        return (SimClock.nanoTime() - startNanos) / 1e9;
    }

    public void resetRuntime() {
        startNanos = SimClock.nanoTime();
    }

    public final void requestOpModeStop() {
        stopRequested = true;
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.update();
    }

    // Not in the SDK; for the simulator.
    public final boolean simStopRequested() {
        return stopRequested;
    }
}
