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

/**
 * FollowerAuto (handoff doc {@code CLAUDE.md} §9.1) — drives the Path/VelocityProfile/Follower
 * stack against the real robot. kA=0 here on purpose: verify the velocity path (signs, kS, kV)
 * before turning on the acceleration feedforward. Every constant below is a PLACEHOLDER from the
 * handoff's §11 table or the sim test's arch path — none of it is measured. Do not trust it,
 * measure it on the bench per §9 steps 6-10 before running this for real.
 */
@Autonomous(name = "Odyssey Follower Auto")
public class OpModeOne extends OpMode {

    // ---- placeholders: measure per handoff §9 steps 6-10, replace before real use ----
    private static final double K_S = 0.1;
    private static final double K_V = 0.00064;
    private static final double K_A = 0; // stays 0 until kA is measured (§9 step 12)
    private static final double LX = 165; // mm, half the wheelbase — MEASURE
    private static final double LY = 165; // mm, half the track width — MEASURE

    private static final double MAX_VELOCITY = 1350;         // mm/s
    private static final double MAX_ACCELERATION = 3000;     // mm/s^2
    private static final double MAX_BRAKE = 3000;             // mm/s^2
    private static final double MAX_CENTRIPETAL_ACCEL = 2500; // mm/s^2
    private static final double PROFILE_STEP = 5;             // mm

    private static final double MIN_DRIVE_SPEED = 110; // mm/s floor, breaks the self-start deadlock (§6.5)
    private static final double FLOOR_CUTOFF = 500;     // mm from the end the floor stops applying

    private static final double TRANS_KP = 1.0, TRANS_KI = 0, TRANS_KD = 0;
    private static final double HEADING_KP = 1.0, HEADING_KI = 0, HEADING_KD = 0;
    private static final double TRANS_OUTPUT_LIMIT = 1000; // mm/s
    private static final double HEADING_OUTPUT_LIMIT = 3;  // rad/s

    private static final double FINISH_TOLERANCE_MM = 20; // stop driving once this close to the end

    private Path path;
    private PinpointLocalizer localizer;
    private VelocityProfile velocityProfile;
    private Follower follower;
    private MecanumDrive mecanumDrive;
    private ElapsedTime timer;
    private boolean finished;

    @Override
    public void init() {
        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "localizer");
        pinpointDriver.resetPosAndIMU(); // robot must be physically still — recalibrates the IMU
        localizer = new PinpointLocalizer(pinpointDriver);

        // PLACEHOLDER PATH — replace with the real autonomous path before running.
        // Mirrors the sim test's symmetric arch: (0,0) heading 90deg -> (1200,0) heading -90deg.
        path = new Path(new BezierCurve(
                new Vector2d(0, 0), new Vector2d(0, 600),
                new Vector2d(1200, 600), new Vector2d(1200, 0),
                Math.toRadians(90), Math.toRadians(-90)));

        velocityProfile = new VelocityProfile(path, MAX_VELOCITY, MAX_ACCELERATION, MAX_BRAKE, MAX_CENTRIPETAL_ACCEL, PROFILE_STEP);

        PIDController pidTranslational = new PIDController(0, TRANS_KP, TRANS_KI, TRANS_KD);
        pidTranslational.setOutputLimits(-TRANS_OUTPUT_LIMIT, TRANS_OUTPUT_LIMIT);

        PIDController pidHeading = new PIDController(0, HEADING_KP, HEADING_KI, HEADING_KD);
        pidHeading.setOutputLimits(-HEADING_OUTPUT_LIMIT, HEADING_OUTPUT_LIMIT);

        mecanumDrive = new MecanumDrive(K_S, K_V, K_A, LX, LY, leftFront, rightFront, leftBack, rightBack, voltageSensor);

        follower = new Follower(path, localizer, velocityProfile, pidTranslational, pidHeading, MIN_DRIVE_SPEED, FLOOR_CUTOFF);

        timer = new ElapsedTime();
        finished = false;
    }

    @Override
    public void start() {
        timer.reset(); // only reset point — the Follower's PIDs derive dt from consecutive calls
    }

    @Override
    public void loop() {
        localizer.update(); // must come first — otherwise the Follower runs on a frozen pose

        if (finished) {
            mecanumDrive.drive(new DriveSignal(0, 0, 0, 0, 0));
        } else {
            DriveSignal signal = follower.update(timer.seconds());
            mecanumDrive.drive(signal);
            if (follower.getDistanceRemaining() <= FINISH_TOLERANCE_MM) {
                finished = true;
            }
        }

        Pose2d pose = localizer.getPose();
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading", pose.getHeading());
        telemetry.addData("Distance remaining", follower.getDistanceRemaining());
        telemetry.addData("Finished", finished);
        telemetry.update();
    }
}
