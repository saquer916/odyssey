package org.firstinspires.ftc.teamcode.odyssey.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.odyssey.localization.GoBildaPinpointDriver;
@TeleOp(name = "kA op mode")
public class kAOpMode extends OpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private ElapsedTime runtime = new ElapsedTime();
    private VoltageSensor voltageSensor;
    private GoBildaPinpointDriver localizer;


    private enum motorState {
        START_MOVE,
        MOVING,
        START_REST,
        RESTING,
        DONE
    }
    private motorState currentState = motorState.START_MOVE;
    private double[] times = new double[300];
    private double[] vels = new double[300];
    private int count = 0;
    private boolean computed = false;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");

        localizer = hardwareMap.get(GoBildaPinpointDriver.class, "localizer");
        localizer.resetPosAndIMU();

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        runtime.reset();
        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);

    }

    @Override
    public void loop() {
        localizer.update();

        double vel = localizer.getVelX(DistanceUnit.MM);
        double time = runtime.milliseconds();

        if (runtime.milliseconds() < 2000) {
            if (count < 300) {
                times[count] = time;
                vels[count] = vel;
                count++;
            }
        }


        if (runtime.milliseconds() >= 2000) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            if (!computed) {
                double sum = 0;
                for (int i = count - 15; i < count; i++) {
                    sum += vels[i];
                }
                double vss = sum / 15;

                double threshold = 0.632 * vss;

                double tau = 0;
                for (int i = 0; i < count; i++) {
                    if (vels[i] >= threshold) {
                        tau = times[i] / 1000.0;
                        break;
                    }
                }

                double kV = 0.000444;
                double kA = kV * tau;

                telemetry.log().add("vss: %.2f", vss);
                telemetry.log().add("tau: %.4f", tau);
                telemetry.log().add("kA: %.6f", kA);

                computed = true;
            }

        }

        telemetry.update();


    }
}
