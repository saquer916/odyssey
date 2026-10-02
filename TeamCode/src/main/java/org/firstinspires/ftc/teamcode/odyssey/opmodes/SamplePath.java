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
import org.firstinspires.ftc.teamcode.odyssey.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.odyssey.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.odyssey.localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.odyssey.path.BezierCurve;
import org.firstinspires.ftc.teamcode.odyssey.path.Path;
import org.firstinspires.ftc.teamcode.odyssey.path.VelocityProfile;
import org.firstinspires.ftc.teamcode.odyssey.path.heading.ConstantInterpolator;
import org.firstinspires.ftc.teamcode.odyssey.path.heading.LinearInterpolator;

// An auto-style path with FirstTest's constants: 600 mm forward, a left curve that turns the robot
// to face +Y, then 400 mm along +Y (~1.6 m). On the robot it needs ~1.3 m clear ahead and ~1 m to
// the left of where it starts.
@Autonomous(name = "Sample Path - Odyssey")
public class SamplePath extends OpMode {

    private static final double kS = 0.11265261;
    private static final double kV = 0.000444;
    private static final double kA = 0.000154;
    private static final double lX = 140.25;
    private static final double lY = 221.75;

    private static final double TRANSLATIONAL_kP = 0.25;
    private static final double TRANSLATIONAL_kI = 0.0;
    private static final double TRANSLATIONAL_kD = 0.001;
    private static final double TRANSLATIONAL_LIMIT = 1900;

    private static final double HEADING_kP = 2.0;
    private static final double HEADING_kI = 0.0;
    private static final double HEADING_kD = 0.001;
    private static final double HEADING_LIMIT = 3.0;
    private static final double MIN_DRIVE_SPEED = 110;
    private static final double FLOOR_CUTOFF = 500;

    private PinpointLocalizer localizer;
    private Path path;
    private Follower follower;
    private MecanumDrive drive;
    private ElapsedTime timer;

    @Override
    public void init() {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "localizer");
        pinpointDriver.resetPosAndIMU();
        localizer = new PinpointLocalizer(pinpointDriver);

        path = new Path(
                new BezierCurve(
                        new Vector2d(0, 0), new Vector2d(200, 0),
                        new Vector2d(400, 0), new Vector2d(600, 0),
                        new ConstantInterpolator(0)),
                new BezierCurve(
                        new Vector2d(600, 0), new Vector2d(850, 0),
                        new Vector2d(1000, 150), new Vector2d(1000, 400),
                        new LinearInterpolator(0, Math.PI / 2)),
                new BezierCurve(
                        new Vector2d(1000, 400), new Vector2d(1000, 533),
                        new Vector2d(1000, 667), new Vector2d(1000, 800),
                        new ConstantInterpolator(Math.PI / 2)));

        VelocityProfile profile = new VelocityProfile(path, 1900, 3000, 3000, 2500, 1.25);

        PIDController pidTranslational = new PIDController(0, TRANSLATIONAL_kP, TRANSLATIONAL_kI, TRANSLATIONAL_kD);
        pidTranslational.setOutputLimits(-TRANSLATIONAL_LIMIT, TRANSLATIONAL_LIMIT);
        PIDController pidHeading = new PIDController(0, HEADING_kP, HEADING_kI, HEADING_kD);
        pidHeading.setOutputLimits(-HEADING_LIMIT, HEADING_LIMIT);

        follower = new Follower(path, localizer, profile, pidTranslational, pidHeading, MIN_DRIVE_SPEED, FLOOR_CUTOFF);
        drive = new MecanumDrive(kS, kV, kA, lX, lY, leftFront, rightFront, leftBack, rightBack, voltageSensor);

        telemetry.addLine("Odyssey SamplePath ready. Needs ~1.3 m clear ahead and ~1 m to the left.");
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
        if (distanceRemaining <= 0) {
            telemetry.addLine("PATH COMPLETE");
        }
        telemetry.update();
    }
}
