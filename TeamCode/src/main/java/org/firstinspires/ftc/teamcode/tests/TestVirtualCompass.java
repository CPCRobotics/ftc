package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutonomousOpmode;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Shows current angle
 */
@Autonomous(name="Test Virtual Compass", group="TestSoftware")
public class TestVirtualCompass extends AutonomousOpmode {
    @Override
    protected void startAutonomous() throws InterruptedException {
        while (opModeIsActive()) {
            Twigger.getInstance()
                    .addData("angle", tilerunner.compass.getAngle())
                    .update();
            Thread.sleep(500);
        }
    }
}
