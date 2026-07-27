package org.firstinspires.ftc.teamcode.odyssey.localization;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.odyssey.geometry.Pose2d;

public class PinpointLocalizer implements Localizer {

    private GoBildaPinpointDriver pinpointDriver;

    public PinpointLocalizer(GoBildaPinpointDriver pinpointDriver) {
        this.pinpointDriver = pinpointDriver;
    }
    @Override
    public Pose2d getPose() {
        double x = pinpointDriver.getPosX(DistanceUnit.MM);
        double y = pinpointDriver.getPosY(DistanceUnit.MM);
        double heading = pinpointDriver.getHeading(AngleUnit.RADIANS);
        return new Pose2d(x, y, heading);
    }

    @Override
    public void update() {
        pinpointDriver.update();
    }
}
