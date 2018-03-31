package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Robot runs in a square. Verifies that the IMU, drive encoders, and Tilerunner is working well.
 */
@Autonomous(name="Test Square", group="Test")
public class TestSquare extends LinearOpMode implements BusyWaitHandler {
    private static final double ROBOT_POWER = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        Tilerunner tilerunner = new Tilerunner();
        tilerunner.init(hardwareMap, telemetry, Tilerunner.OpmodeType.AUTONOMOUS);


        waitForStart();

        while (isActive()) {
            tilerunner.move(this, ROBOT_POWER, 12);
            tilerunner.turn(this, ROBOT_POWER, 90);
        }
    }

    @Override
    public boolean isActive() {
        Twigger.getInstance().update();
        return opModeIsActive();
    }
}
