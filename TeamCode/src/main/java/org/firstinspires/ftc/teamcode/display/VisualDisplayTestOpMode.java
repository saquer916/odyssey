package org.firstinspires.ftc.teamcode.display;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import team.techtigers.core.display.AdafruitNeoPixel;
import team.techtigers.core.display.VisualDisplay;

@TeleOp(name="Visual Display Op Mode", group="Test")
public class VisualDisplayTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AdafruitNeoPixel driver = hardwareMap.get(AdafruitNeoPixel.class, "visual_display");
        driver.initialize(64, 3);
        TestView view = new TestView();
        VisualDisplay visualDisplay = new VisualDisplay(driver, view);

        waitForStart();

        while(opModeIsActive()) {
            visualDisplay.update();
        }
    }
}
