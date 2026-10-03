package org.firstinspires.ftc.teamcode.odyssey.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.odyssey.control.PIDController;
import org.firstinspires.ftc.teamcode.odyssey.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.odyssey.follower.DriveSignal;
import org.firstinspires.ftc.teamcode.odyssey.follower.Follower;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.odyssey.localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;
import org.firstinspires.ftc.teamcode.odyssey.path.Path;
import org.firstinspires.ftc.teamcode.odyssey.path.VelocityProfile;
import org.firstinspires.ftc.teamcode.odyssey.path.heading.ConstantInterpolator;
import org.firstinspires.ftc.teamcode.odyssey.path.heading.LinearInterpolator;

@Autonomous(name = "First Test - Odyssey")
public class FirstTest extends OpMode {

    private static final double kS = 0.11265261;
    private static final double kV = 0.000444;
    private static final double kA = 0.000154;
    private static final double lX = 140.25;
    private static final double lY = 221.75;

    // Tuned in odyssey-sim on several paths, with a tired/heavy and a fresh/light robot, noisy
    // odometry and a ~30 ms loop. Measured constants above aren't tuned.
    private static final double TRANSLATIONAL_kP = 30;
    private static final double TRANSLATIONAL_kI = 8.1;
    private static final double TRANSLATIONAL_kD = 2.0;
    private static final double TRANSLATIONAL_LIMIT = 1900;

    private static final double HEADING_kP = 13.0;
    private static final double HEADING_kI = 0.0;
    private static final double HEADING_kD = 0.066;
    private static final double HEADING_LIMIT = 1.32;

    // MAX_VELOCITY leaves headroom below the motors' ~2250 mm/s so corrections don't saturate.
    private static final double MAX_VELOCITY = 890;
    private static final double MAX_ACCEL = 3160;
    private static final double MAX_BRAKE = 560;
    private static final double MAX_CENTRIPETAL = 2275;
    // Keep MIN_DRIVE_SPEED above (kA / kV) * MAX_BRAKE (~0.35 * MAX_BRAKE): below that, the braking
    // feedforward cancels the floor near the end and the robot can stall short of it.
    private static final double MIN_DRIVE_SPEED = 255;
    private static final double FLOOR_CUTOFF = 0;

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private VoltageSensor voltageSensor;

    private PinpointLocalizer localizer;
    private Path path;
    private VelocityProfile profile;
    private PIDController pidTranslational;
    private PIDController pidHeading;
    private Follower follower;
    private MecanumDrive drive;
    private ElapsedTime timer;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "localizer");
        pinpointDriver.resetPosAndIMU();
        localizer = new PinpointLocalizer(pinpointDriver);
        localizer.setPose(new Pose2d(0.1, 0.1, 0));
        path = new Path(
                new BezierCurve(
                        new Vector2d(0, 0),
                        new Vector2d(0, 0),
                        new Vector2d(0, 100),
                        new Vector2d(0, 100),
                        new ConstantInterpolator(0)
                )
        );

        profile = new VelocityProfile(path, MAX_VELOCITY, MAX_ACCEL, MAX_BRAKE, MAX_CENTRIPETAL, 1.25);

        pidTranslational = new PIDController(0, TRANSLATIONAL_kP, TRANSLATIONAL_kI, TRANSLATIONAL_kD);
        pidTranslational.setOutputLimits(-TRANSLATIONAL_LIMIT, TRANSLATIONAL_LIMIT);

        pidHeading = new PIDController(0, HEADING_kP, HEADING_kI, HEADING_kD);
        pidHeading.setOutputLimits(-HEADING_LIMIT, HEADING_LIMIT);

        follower = new Follower(path, localizer, profile, pidTranslational, pidHeading, MIN_DRIVE_SPEED, FLOOR_CUTOFF);

        drive = new MecanumDrive(kS, kV, kA, lX, lY, leftFront, rightFront, leftBack, rightBack, voltageSensor);

        telemetry.addLine("Odyssey FirstTest ready. Robot must be at (600, 600, 0 deg).");
        telemetry.update();
    }

    @Override
    public void start() {
        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        localizer.update();
        DriveSignal signal = follower.update(timer.seconds());
        drive.drive(signal);

        double distanceRemaining = follower.getDistanceRemaining();
        telemetry.addData("Pose", localizer.getPose());
        telemetry.addData("Distance Remaining", distanceRemaining);
        telemetry.addData("Forward Vel Cmd", signal.getForwardVelocity());
        telemetry.addData("Strafe Vel Cmd", signal.getStrafeVelocity());
        telemetry.addData("Turn Cmd", signal.getTurn());
        if (distanceRemaining <= 0) {
            telemetry.addLine("PATH COMPLETE");
        }
        telemetry.update();
    }
}
