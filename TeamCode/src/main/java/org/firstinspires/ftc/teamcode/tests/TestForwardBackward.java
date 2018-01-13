package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Checks to see if moving forward and backward behave as expected
 */
@Autonomous(name="Test Forward-Backward", group="TestHardware")
public class TestForwardBackward extends LinearOpMode implements BusyWaitHandler {
    @Override
    public void runOpMode() throws InterruptedException {
        Tilerunner tilerunner = new Tilerunner();
        tilerunner.init(hardwareMap, telemetry);
        Twigger.getInstance().init(telemetry);

        waitForStart();

        while (opModeIsActive()) {
            tilerunner.move(this, 1, 10);
            Thread.sleep(100);
            tilerunner.move(this, 1, -10);
            Thread.sleep(100);
        }
    }

    @Override
    public boolean isActive() {
        Twigger.getInstance().update();
        return opModeIsActive();
    }
}
