package com.qualcomm.robotcore.hardware;

// odyssey-sim stand-in.
public interface HardwareDevice {
    default String getDeviceName() { return getClass().getSimpleName(); }
    default String getConnectionInfo() { return "odyssey-sim"; }
    default int getVersion() { return 1; }
    default void resetDeviceConfigurationForOpMode() {}
    default void close() {}
}
