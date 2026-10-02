package org.firstinspires.ftc.teamcode.odysseysim;

// Thrown out of a hardware call when an OpMode is still running sim.stopGraceSeconds after stop,
// the way the SDK would kill it. An Error so OpMode code catching Exception doesn't swallow it.
final class SimStop extends Error {
    SimStop() {
        super("odyssey-sim: OpMode didn't stop in time", null, false, false);
    }
}
