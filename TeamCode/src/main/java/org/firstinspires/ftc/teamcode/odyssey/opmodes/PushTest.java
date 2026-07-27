package org.firstinspires.ftc.teamcode.odyssey.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.odyssey.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.odyssey.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.odyssey.localization.PinpointLocalizer;

@TeleOp(name = "Push Test Odometry")
public class PushTest extends OpMode {
    private PinpointLocalizer localizer;
    @Override
    public void init() {
        GoBildaPinpointDriver goBildaPinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "localizer");
        goBildaPinpointDriver.resetPosAndIMU();
        localizer = new PinpointLocalizer(goBildaPinpointDriver);
    }

    @Override
    public void loop() {
        localizer.update();
        Pose2d pose = localizer.getPose();
        telemetry.addData("X: ", pose.getX());
        telemetry.addData("Y: ", pose.getY());
        telemetry.addData("HEADING: ", pose.getHeading());
        telemetry.update();
    }
}
