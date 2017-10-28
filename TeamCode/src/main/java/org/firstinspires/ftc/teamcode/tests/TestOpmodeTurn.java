package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Created by samuel on 10/27/17.
 */

@Autonomous(name="Test Turn", group="Test")
public class TestOpmodeTurn extends LinearOpMode implements BusyWaitHandler {
    @Override
    public void runOpMode() throws InterruptedException {
        Tilerunner tilerunner = new Tilerunner();
        tilerunner.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            tilerunner.turn(this, 1, 180);
            Thread.sleep(500);
            tilerunner.turn(this, -1, 180);
            Thread.sleep(500);
        }
    }

    @Override
    public boolean isActive() {
        Twigger.getInstance().update();
        return opModeIsActive();
    }
}
