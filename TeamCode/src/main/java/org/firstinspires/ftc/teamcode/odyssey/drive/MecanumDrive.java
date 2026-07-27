package org.firstinspires.ftc.teamcode.odyssey.drive;

import static org.firstinspires.ftc.teamcode.odyssey.utils.MathUtils.max;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.odyssey.follower.DriveSignal;

public class MecanumDrive {
    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightBack;
    private final double kS;
    private final double kV;
    private final double kA;
    private final double k;
    private final VoltageSensor voltageSensor;
    public MecanumDrive(double kS, double kV, double kA, double lX, double lY, DcMotorEx leftFront, DcMotorEx rightFront, DcMotorEx leftBack, DcMotorEx rightBack, VoltageSensor voltageSensor) {
        this.rightFront = rightFront;
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.k = lX + lY;
        this.voltageSensor = voltageSensor;

        // RUN_WITHOUT_ENCODER: the kS/kV/kA math models power, not the hub's velocity PID.
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // BRAKE: FLOAT would let the robot drift past the endpoint after the last setPower(0).
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Right side is mirror-mounted on a standard symmetric FTC chassis, so it spins the
        // opposite way for the same positive power. REVERSE cancels that so all-four-positive
        // drives forward. VERIFY ON THE BENCH (handoff §9 step 8) — if the robot spins in place
        // instead of driving straight, this assumption is backwards for this chassis.
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(DriveSignal signal) {
        double f = signal.getForwardVelocity();
        double s = signal.getStrafeVelocity();
        double w = signal.getTurn();

        double v_FL = f - s - k*w;
        double v_FR = f + s + k*w;
        double v_BL = f + s - k*w;
        double v_BR = f - s + k*w;

        double af = signal.getForwardAcceleration();
        double as = signal.getStrafeAcceleration();

        double a_FL = af - as;
        double a_FR = af + as;
        double a_BL = af + as;
        double a_BR = af - as;

        double voltageComp = 12.0 / voltageSensor.getVoltage();

        double p_FL = (kS * Math.signum(v_FL) + kV * v_FL + kA * a_FL) * voltageComp;
        double p_FR = (kS * Math.signum(v_FR) + kV * v_FR + kA * a_FR) * voltageComp;
        double p_BL = (kS * Math.signum(v_BL) + kV * v_BL + kA * a_BL) * voltageComp;
        double p_BR = (kS * Math.signum(v_BR) + kV * v_BR + kA * a_BR) * voltageComp;

        double max = max(Math.abs(p_FL), Math.abs(p_FR), Math.abs(p_BL), Math.abs(p_BR));
        if (max > 1.0) {
            p_FL /= max;
            p_FR /= max;
            p_BL /= max;
            p_BR /= max;
        }

        leftFront.setPower(p_FL);
        rightFront.setPower(p_FR);
        leftBack.setPower(p_BL);
        rightBack.setPower(p_BR);
    }
}
