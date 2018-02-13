package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Ensures the encoders are in working order.
 *
 * The encoders are occasionally wired in reverse, which completely breaks the Autonomous period.
 */
@Autonomous(name="Encoder Checker")
@Disabled
public class OpmodeEncoderCheck extends LinearOpMode {
    private Tilerunner tilerunner = new Tilerunner();
    private String mode = "STOPPED";

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