package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Created by samuel on 10/21/17.
 */
@Autonomous(name="Encoder Checker")
public class OpmodeEncoderCheck extends LinearOpMode {
    Tilerunner tilerunner = new Tilerunner();
    String mode = "STOPPED";

    @Override
    public void runOpMode() throws InterruptedException {
        tilerunner.init(hardwareMap, telemetry);
        composeLogging();

        waitForStart();

        mode = "MOVING";
        tilerunner.motorPair.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilerunner.motorPair.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tilerunner.motorPair.setPower(1);
        for (int i = 0; i < 100; i++) {
            if (!opModeIsActive())
                return;
            Thread.sleep(10);
        }

        if (tilerunner.leftMotor.getCurrentPosition() > 0 && tilerunner.rightMotor.getCurrentPosition() > 0) {
            Twigger.getInstance().sendOnce("Encoders are Good!");
        } else {
            Twigger.getInstance().sendOnce("Encoders are Bad!");
        }

        while (opModeIsActive())
            Thread.sleep(10);

    }

    private void composeLogging() {
        Twigger.getInstance()
                .addData("mode", new Func<String>() {
                    @Override
                    public String value() {
                        return mode;
                    }
                })
                .addData("position", new Func<String>() {
                    @Override
                    public String value() {
                        return String.valueOf(tilerunner.leftMotor.getCurrentPosition());
                    }
                });
    }
}