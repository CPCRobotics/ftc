package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by samuel on 8/11/17
 */

@Autonomous( name="SquareAuto", group="Iterative OpMode")
public class SquareAuto extends LinearOpMode implements BusyWaitHandler {

    private HardwareTilerunner tileRunner = new HardwareTilerunner();

    @Override
    public void runOpMode() throws InterruptedException {
        tileRunner.init(hardwareMap);

        // Set up our telemetry dashboard
        composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        tileRunner.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive()) {
            moveEdge();
        }

    }

    private void moveEdge() {
        TilerunnerUtils.move(tileRunner, this, 36, 1);
        TilerunnerUtils.turn(tileRunner, this, 1, 90);
    }

    private void composeTelemetry() {}

    @Override
    public boolean isActive() {
        telemetry.addData("current position (left)", tileRunner.leftMotor.getCurrentPosition());
        telemetry.addData("current position (right)", tileRunner.rightMotor.getCurrentPosition());
        telemetry.addData("target position", tileRunner.leftMotor.getTargetPosition());
        telemetry.addData("heading: ", tileRunner.getHeading());
        telemetry.update();

        return opModeIsActive();
    }
}
