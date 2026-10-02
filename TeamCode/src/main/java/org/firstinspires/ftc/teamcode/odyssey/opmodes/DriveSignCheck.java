package org.firstinspires.ftc.teamcode.odyssey.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.odyssey.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.odyssey.follower.DriveSignal;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odyssey.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.odyssey.localization.PinpointLocalizer;

// Drives through MecanumDrive (the same code path the Follower uses) one axis at a time and checks
// the Pinpoint saw matching motion: +forward -> +X, +strafe -> +Y (left), +turn -> heading rises.
// Needs ~0.5 m of clear floor in front of and to the left of the robot.
@TeleOp(name = "Drive Sign Check")
public class DriveSignCheck extends OpMode {
    private static final double kS = 0.11265261;
    private static final double kV = 0.000444;
    private static final double kA = 0.000154;
    private static final double lX = 140.25;
    private static final double lY = 221.75;

    private static final double TEST_SPEED = 300;    // mm/s
    private static final double TEST_TURN = 1.0;     // rad/s
    private static final double DRIVE_MS = 500;
    private static final double SETTLE_MS = 1000;
    private static final double MIN_TRAVEL = 20;     // mm
    private static final double MIN_ROTATION = 0.1;  // rad

    private enum Step {
        FORWARD,
        STRAFE_LEFT,
        TURN_CCW,
        DONE
    }
    private Step currentStep = Step.FORWARD;
    private boolean driving = true;

    private PinpointLocalizer localizer;
    private MecanumDrive drive;
    private final ElapsedTime runtime = new ElapsedTime();
    private Pose2d stepStart;

    @Override
    public void init() {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "localizer");
        pinpointDriver.resetPosAndIMU();
        localizer = new PinpointLocalizer(pinpointDriver);

        drive = new MecanumDrive(kS, kV, kA, lX, lY, leftFront, rightFront, leftBack, rightBack,
                hardwareMap.voltageSensor.iterator().next());

        telemetry.addLine("Drive Sign Check ready. Needs ~0.5 m clear in front and to the left.");
        telemetry.update();
    }

    @Override
    public void start() {
        localizer.update();
        stepStart = localizer.getPose();
        runtime.reset();
    }

    @Override
    public void loop() {
        localizer.update();
        Pose2d pose = localizer.getPose();

        if (currentStep != Step.DONE) {
            if (driving) {
                drive.drive(signalFor(currentStep));
                if (runtime.milliseconds() >= DRIVE_MS) {
                    drive.drive(new DriveSignal(0, 0, 0, 0, 0));
                    driving = false;
                    runtime.reset();
                }
            } else if (runtime.milliseconds() >= SETTLE_MS) {
                report(currentStep, pose.relativeTo(stepStart));
                currentStep = Step.values()[currentStep.ordinal() + 1];
                stepStart = pose;
                driving = true;
                runtime.reset();
            }
        }

        telemetry.addData("Step", currentStep);
        telemetry.addData("Pose", pose);
        telemetry.update();
    }

    private DriveSignal signalFor(Step step) {
        switch (step) {
            case FORWARD:
                return new DriveSignal(TEST_SPEED, 0, 0, 0, 0);
            case STRAFE_LEFT:
                return new DriveSignal(0, TEST_SPEED, 0, 0, 0);
            case TURN_CCW:
                return new DriveSignal(0, 0, 0, 0, TEST_TURN);
            default:
                return new DriveSignal(0, 0, 0, 0, 0);
        }
    }

    // delta is in the robot frame at the start of the step, so +X is the way the robot was facing.
    private void report(Step step, Pose2d delta) {
        boolean pass;
        switch (step) {
            case FORWARD:
                pass = delta.getX() > MIN_TRAVEL && Math.abs(delta.getY()) < delta.getX();
                break;
            case STRAFE_LEFT:
                pass = delta.getY() > MIN_TRAVEL && Math.abs(delta.getX()) < delta.getY();
                break;
            case TURN_CCW:
                pass = delta.getHeading() > MIN_ROTATION;
                break;
            default:
                return;
        }
        telemetry.log().add("%s %s | dx: %.1f dy: %.1f dh: %.3f",
                step, pass ? "PASS" : "FAIL", delta.getX(), delta.getY(), delta.getHeading());
    }
}
