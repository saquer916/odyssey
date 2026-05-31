package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Ball Detection TeleOp", group = "Vision")
public class OpenCvBallDetectionOpMode extends OpMode {

    private static final Scalar LOW_GREEN  = new Scalar(50,  100, 80);
    private static final Scalar HIGH_GREEN = new Scalar(85,  255, 255);

    private static final Scalar LOW_PURPLE  = new Scalar(125, 100, 60);
    private static final Scalar HIGH_PURPLE = new Scalar(165, 255, 255);

    private static final String LABEL_GREEN  = "Green";
    private static final String LABEL_PURPLE = "Purple";

    private static final Scalar COLOR_GREEN  = new Scalar(0,   255, 0);
    private static final Scalar COLOR_PURPLE = new Scalar(148, 0,   211);

    private static final double MIN_BALL_AREA   = 3000.0;
    private static final double CLOSE_BALL_AREA = 15000.0;

    private static final double FRAME_CENTER_X  = 320.0;
    private static final double CENTER_DEADBAND = 20.0;
    private static final double MAX_STRAFE_POWER = 0.5;
    private static final double DRIVE_POWER      = 0.5;

    private OpenCvWebcam camera;
    private TrcOpenCvColorBlobPipeline pipeline;

    private TrcOpenCvColorBlobPipeline.DetectedObject[] lastDetectedObjects =
            new TrcOpenCvColorBlobPipeline.DetectedObject[0];

    private volatile boolean cameraReady = false;

    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;
    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;

    @Override
    public void init() {
        backLeft   = hardwareMap.get(DcMotor.class, "left_back");
        backRight  = hardwareMap.get(DcMotor.class, "right_back");
        frontLeft  = hardwareMap.get(DcMotor.class, "left_front");
        frontRight = hardwareMap.get(DcMotor.class, "right_front");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        TrcOpenCvColorBlobPipeline.PipelineParams params =
                new TrcOpenCvColorBlobPipeline.PipelineParams()
                        .addColorThresholds(
                                LABEL_GREEN, true,
                                new double[]{LOW_GREEN.val[0],  LOW_GREEN.val[1],  LOW_GREEN.val[2]},
                                new double[]{HIGH_GREEN.val[0], HIGH_GREEN.val[1], HIGH_GREEN.val[2]})
                        .addColorThresholds(
                                LABEL_PURPLE, true,
                                new double[]{LOW_PURPLE.val[0],  LOW_PURPLE.val[1],  LOW_PURPLE.val[2]},
                                new double[]{HIGH_PURPLE.val[0], HIGH_PURPLE.val[1], HIGH_PURPLE.val[2]})
                        .buildColorThresholdSets()
                        .setColorConversion(TrcOpenCvColorBlobPipeline.ColorConversion.RGBToHSV)
                        .setMorphology(true, 5)
                        .setFilterContourParams(true,
                                new TrcOpenCvColorBlobPipeline.FilterContourParams()
                                        .setMinArea(500));

        pipeline = new TrcOpenCvColorBlobPipeline("BallDetection", params, null);

        camera.setPipeline(new OpenCvPipeline() {
            private final Mat rgbMat = new Mat();

            @Override
            public Mat processFrame(Mat input) {
                Imgproc.cvtColor(input, rgbMat, Imgproc.COLOR_RGBA2RGB);

                TrcOpenCvColorBlobPipeline.DetectedObject[] detected = pipeline.process(rgbMat);

                if (detected != null) {
                    for (TrcOpenCvColorBlobPipeline.DetectedObject obj : detected) {
                        if (obj.objArea > MIN_BALL_AREA) {
                            Scalar boxColor = LABEL_GREEN.equals(obj.label) ? COLOR_GREEN : COLOR_PURPLE;
                            Imgproc.rectangle(rgbMat, obj.objRect.tl(), obj.objRect.br(), boxColor, 3);
                            String text = obj.label + " (" + (int) obj.objArea + "px)";
                            Point textPos = new Point(obj.objRect.x, Math.max(obj.objRect.y - 8, 12));
                            Imgproc.putText(rgbMat, text, textPos, Imgproc.FONT_HERSHEY_SIMPLEX, 0.55, boxColor, 2);
                            Point center = new Point(
                                    obj.objRect.x + obj.objRect.width  / 2.0,
                                    obj.objRect.y + obj.objRect.height / 2.0);
                            Imgproc.circle(rgbMat, center, 4, boxColor, -1);
                        }
                    }
                }

                return rgbMat;
            }
        });

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                cameraReady = true;
                telemetry.addLine("Camera streaming started");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        telemetry.addLine("initialized waiting for cam");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (!cameraReady) {
            stopMotors();
            telemetry.addLine("waiting for cam");
            telemetry.update();
            return;
        }

        TrcOpenCvColorBlobPipeline.DetectedObject[] fresh = pipeline.getDetectedObjects();
        if (fresh != null) {
            lastDetectedObjects = fresh;
        }

        List<TrcOpenCvColorBlobPipeline.DetectedObject> greenBalls  = new ArrayList<>();
        List<TrcOpenCvColorBlobPipeline.DetectedObject> purpleBalls = new ArrayList<>();

        for (TrcOpenCvColorBlobPipeline.DetectedObject obj : lastDetectedObjects) {
            if (obj.objArea < MIN_BALL_AREA) continue;
            if (LABEL_GREEN.equals(obj.label))        greenBalls.add(obj);
            else if (LABEL_PURPLE.equals(obj.label))  purpleBalls.add(obj);
        }

        telemetry.addData("Green balls detected",  greenBalls.size());
        telemetry.addData("Purple balls detected", purpleBalls.size());

        if (!greenBalls.isEmpty()) {
            trackBall(greenBalls.get(0), LABEL_GREEN);
        } else if (!purpleBalls.isEmpty()) {
            trackBall(purpleBalls.get(0), LABEL_PURPLE);
        } else {
            stopMotors();
            telemetry.addLine("No ball detected");
        }

        telemetry.update();
    }

    private void trackBall(TrcOpenCvColorBlobPipeline.DetectedObject ball, String color) {
        double centerX = ball.objRect.x + ball.objRect.width / 2.0;
        double error   = centerX - FRAME_CENTER_X;

        telemetry.addData("  " + color + " centerX", "%.0f", centerX);
        telemetry.addData("  " + color + " area",    "%.0f", ball.objArea);

        if (Math.abs(error) > CENTER_DEADBAND) {
            double strafePower = MAX_STRAFE_POWER * (error / FRAME_CENTER_X);
            strafePower = Math.max(-MAX_STRAFE_POWER, Math.min(MAX_STRAFE_POWER, strafePower));

            backLeft.setPower(strafePower);
            frontLeft.setPower(-strafePower);
            backRight.setPower(-strafePower);
            frontRight.setPower(strafePower);

            telemetry.addData("  Strafing", "%.2f", strafePower);

        } else if (ball.objArea < CLOSE_BALL_AREA) {
            backLeft.setPower(DRIVE_POWER);
            frontLeft.setPower(DRIVE_POWER);
            backRight.setPower(DRIVE_POWER);
            frontRight.setPower(DRIVE_POWER);
            telemetry.addLine("  " + color + " centered");

        } else {
            stopMotors();
            telemetry.addLine("  " + color + " stop");
        }
    }

    @Override
    public void stop() {
        stopMotors();
        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }

    private void stopMotors() {
        backLeft.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
    }
}
