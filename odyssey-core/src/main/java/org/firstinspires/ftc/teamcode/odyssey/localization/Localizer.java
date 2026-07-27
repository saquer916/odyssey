package org.firstinspires.ftc.teamcode.odyssey.localization;


import org.firstinspires.ftc.teamcode.odyssey.geometry.Pose2d;

public interface Localizer {
    Pose2d getPose();
    void update();

}
