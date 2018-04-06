package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.EasyLinearOpmode;

/**
 * Checks to see if moving forward and backward behave as expected
 */
@Autonomous(name="Test Forward-Backward", group="TestHardware")
public class TestForwardBackward extends EasyLinearOpmode {

    @Override
    protected void startAutonomous() throws InterruptedException {
        while (opModeIsActive()) {
            tilerunner.moveInches(this, 1, 10);
            Thread.sleep(100);
            tilerunner.moveInches(this, 1, -10);
            Thread.sleep(100);
        }
    }
}
