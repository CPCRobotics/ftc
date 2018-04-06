package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Telemetries encoder data as the robot moves forward
 */
@Autonomous(name="Test Move", group="TestHardware")
@Disabled
public class TestOpmodeMove extends LinearOpMode implements BusyWaitHandler {

    @Override
    public void runOpMode() throws InterruptedException {
        final Tilerunner tilerunner = new Tilerunner();
        tilerunner.init(hardwareMap, telemetry, Tilerunner.OpmodeType.AUTONOMOUS);
        ElapsedTime time = new ElapsedTime();

        Twigger.getInstance()
                .addLine("TunePIDMove")
                    .addData("left", () -> String.valueOf(tilerunner.leftMotor.getCurrentPosition()))
                    .addData("right", () -> String.valueOf(tilerunner.rightMotor.getCurrentPosition()));

        waitForStart();

        tilerunner.motorPair.setPower(1);
        while (isActive() && time.seconds() < 10) {
            Thread.sleep(10);
        }
        Twigger.getInstance().sendOnce("TEST COMPLETE");
    }

    @Override
    public boolean isActive() {
        Twigger.getInstance().update();
        return opModeIsActive();
    }
}
