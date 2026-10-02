package com.qualcomm.robotcore.eventloop.opmode;

import org.firstinspires.ftc.teamcode.odysseysim.Simulation;

// odyssey-sim stand-in. Simulated time moves when the OpMode does hardware I/O, sleeps, idles, or
// checks opModeIsActive() / isStopRequested() / isStarted(). A loop that does none of those (e.g.
// busy-waiting on an ElapsedTime) never sees time pass; the simulator reports where it's stuck.
public abstract class LinearOpMode extends OpMode {

    public abstract void runOpMode() throws InterruptedException;

    @Override public final void init() {}
    @Override public final void init_loop() {}
    @Override public final void start() {}
    @Override public final void loop() {}
    @Override public final void stop() {}

    public void waitForStart() {
        Simulation.current().waitForStart();
    }

    public final boolean opModeIsActive() {
        return Simulation.current().opModeIsActive();
    }

    public final boolean opModeInInit() {
        return Simulation.current().opModeInInit();
    }

    public final boolean isStarted() {
        return Simulation.current().isStarted();
    }

    public final boolean isStopRequested() {
        return Simulation.current().isStopRequested();
    }

    public final void idle() {
        Simulation.current().idle();
    }

    public final void sleep(long milliseconds) {
        Simulation.current().sleep(milliseconds / 1000.0);
    }
}
