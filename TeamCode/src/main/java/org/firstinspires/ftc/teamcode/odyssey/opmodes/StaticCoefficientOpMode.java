package org.firstinspires.ftc.teamcode.odyssey.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.odyssey.localization.GoBildaPinpointDriver;
@TeleOp(name = "static coeff op mode")
public class StaticCoefficientOpMode extends OpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private ElapsedTime runtime = new ElapsedTime();
    private VoltageSensor voltageSensor;
    private GoBildaPinpointDriver localizer;
    private double power = 0.1;
    private double finalVelocity = 0.0;


    private enum motorState {
        START_MOVE,
        MOVING,
        START_REST,
        RESTING,
        DONE
    }
    private motorState currentState = motorState.START_MOVE;

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
    public void loop() {
        localizer.update();
        switch (currentState) {
            case START_MOVE:
                frontLeft.setPower(power);
                frontRight.setPower(power);

                backLeft.setPower(power);
                backRight.setPower(power);

                runtime.reset();

                currentState = motorState.MOVING;
                break;

            case MOVING:
                if (runtime.milliseconds() >= 1750) {
                    finalVelocity = localizer.getVelX(DistanceUnit.MM);

                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);

                    telemetry.log().add("power: %.4f | velo: %.4f", power, finalVelocity);

                    currentState = motorState.START_REST;
                }
                break;

            case START_REST:
                runtime.reset();
                currentState = motorState.RESTING;
                break;

            case RESTING:
                if (runtime.milliseconds() >= 1750) {
                    power += 0.1;
                    currentState = (power > 0.31) ? motorState.DONE : motorState.START_MOVE;
                }
                break;

            case DONE:
                break;
        }
        telemetry.update();
    }
}
