package com.qualcomm.hardware.gobilda;

import org.firstinspires.ftc.teamcode.odysseysim.SimPinpointDriver;

// odyssey-sim stand-in for the Pinpoint driver that ships in the FTC SDK (10.2+), so OpModes that
// use it instead of TeamCode's copy run too.
public class GoBildaPinpointDriver extends SimPinpointDriver {
    public static final byte DEFAULT_ADDRESS = 0x31;
}
