package com.qualcomm.robotcore.hardware;

// odyssey-sim stand-in.
public interface DcMotor extends DcMotorSimple {
    enum ZeroPowerBehavior { UNKNOWN, BRAKE, FLOAT }

    enum RunMode {
        RUN_WITHOUT_ENCODER, RUN_USING_ENCODER, RUN_TO_POSITION, STOP_AND_RESET_ENCODER;

        public boolean isPIDMode() {
            return this == RUN_USING_ENCODER || this == RUN_TO_POSITION;
        }
    }

    void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior);
    ZeroPowerBehavior getZeroPowerBehavior();
    void setPowerFloat();
    boolean getPowerFloat();
    void setTargetPosition(int position);
    int getTargetPosition();
    boolean isBusy();
    int getCurrentPosition();
    void setMode(RunMode mode);
    RunMode getMode();
    int getPortNumber();
}
