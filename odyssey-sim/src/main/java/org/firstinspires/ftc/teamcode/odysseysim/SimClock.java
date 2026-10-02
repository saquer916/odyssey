package org.firstinspires.ftc.teamcode.odysseysim;

// Simulated time. Everything an OpMode reads time from (ElapsedTime, getRuntime) goes through here,
// so runs are repeatable and don't depend on how fast the computer is.
public final class SimClock {
    private static volatile long nanos;

    private SimClock() {}

    public static long nanoTime() {
        return nanos;
    }

    static void set(double seconds) {
        nanos = Math.round(seconds * 1e9);
    }
}
