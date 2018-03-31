package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Makes the robot turn to the right and left 180°. Verifies that the IMU and Tilerunner is working
 * well.
 */
@Autonomous(name="Test Turn", group="TestConcept")
public class TestOpmodeTurn extends LinearOpMode implements BusyWaitHandler {
    private final double SPEED = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Tilerunner tilerunner = new Tilerunner();
        tilerunner.init(hardwareMap, telemetry, Tilerunner.OpmodeType.AUTONOMOUS);

        waitForStart();

        while (opModeIsActive()) {
            tilerunner.turn(this, SPEED, 180);
            Thread.sleep(1000);
            tilerunner.turn(this, SPEED, -180);
            Thread.sleep(1000);
        }
    }

    @Override
    public boolean isActive() {
        Twigger.getInstance().update();
        return opModeIsActive();
    }
}
