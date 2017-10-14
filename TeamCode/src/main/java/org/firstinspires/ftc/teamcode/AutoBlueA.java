package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Created by samuel on 10/14/17.
 */

@Autonomous(name="AutoBlueA", group="Competition")
public class AutoBlueA extends LinearOpMode implements BusyWaitHandler {

    Tilerunner tilerunner = new Tilerunner();
    String mode = "Init";

    @Override
    public void runOpMode() throws InterruptedException {
        tilerunner.init(hardwareMap, telemetry);

        waitForStart();

        mode = "Calibrate";
        //tilerunner.calibrate(this);

        mode = "GO!";
        tilerunner.move(this, 1, 33.5);
        tilerunner.turn(this, 1, 90);
        tilerunner.move(this, 1, 49);
        tilerunner.turn(this, -1, 180);
        tilerunner.move(this, 1, 49);
    }

    private void composeTelemetry() {
        Twigger.getInstance()
                .addData("mode", new Func<String>() {
                    @Override
                    public String value() {
                        return mode;
                    }
                });
    }

    @Override
    public boolean isActive() {
        Twigger.getInstance().update();
        return opModeIsActive();
    }
}
