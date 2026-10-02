package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// odyssey-sim stand-in. The hub's velocity / position PID isn't simulated.
public interface DcMotorEx extends DcMotor {
    void setMotorEnable();
    void setMotorDisable();
    boolean isMotorEnabled();
    void setVelocity(double angularRate);
    void setVelocity(double angularRate, AngleUnit unit);
    double getVelocity();
    double getVelocity(AngleUnit unit);
}
