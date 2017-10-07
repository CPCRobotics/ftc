package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by samuel on 8/11/17
 */

@Autonomous( name="SquareAuto", group="Iterative OpMode")
public class SquareAuto extends LinearOpMode implements BusyWaitHandler{

    private Tilerunner tileRunner = new Tilerunner();

    private String mode = "Stopped";

    @Override
    public void runOpMode() throws InterruptedException {
        tileRunner.init(hardwareMap, telemetry);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        tileRunner.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        mode = "CALIBRATE";
        tileRunner.calibrate();
        mode = "RIGHT";
        tileRunner.turn(this, 1, 180);
        mode = "DONE";
        telemetry.update();

        while (isActive()) {
            Thread.sleep(10);
        }

    }

    private void composeTelemetry() {

        telemetry.setAutoClear(false);

        telemetry.addData("heading", new Func<Double>() {
            @Override
            public Double value() {
                return tileRunner.getHeading();
            }
        });

        telemetry.addData("mode", new Func<String>() {
            @Override
            public String value() {
                return mode;
            }
        });

        telemetry.addLine()
                .addData("imucalib", new Func<Object>() {
                    @Override
                    public Object value() {
                        return String.format("%x", tileRunner.imu.getCalibrationStatus().calibrationStatus);
                    }
                })
                .addData("imustatus", new Func<Object>() {
                    @Override
                    public Object value() {
                        return tileRunner.imu.getSystemStatus().toShortString();
                    }
                });

    }

    @Override
    public boolean isActive() {
        telemetry.update();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }


        return opModeIsActive();
    }

    @Override
    public void sleep(int millis) {
        super.sleep(millis);
    }
}
