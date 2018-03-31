package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutonomousOpmode;

/**
 * Displays the Robot status lights in order: red, green
 */
@Autonomous(name="Test Lights", group="TestHardware")
public class TestLights extends AutonomousOpmode {

    @Override
    protected void startAutonomous() throws InterruptedException {
        final long SLEEP_DELAY = 500;

        while (opModeIsActive()) {
            tilerunner.setLights(false, false);
            Thread.sleep(SLEEP_DELAY);
            tilerunner.setRed(true);
            Thread.sleep(SLEEP_DELAY);
            tilerunner.setGreen(true);
            Thread.sleep(SLEEP_DELAY);
        }
    }
}
