package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by samuel on 8/11/17.
 */

class TilerunnerUtils {
    private TilerunnerUtils() {}

    static void move(HardwareTilerunner tileRunner, BusyWaitHandler waitHandler, double inches, double power) {
        int ticks = (int)(HardwareTilerunner.TICKS_PER_REVOLUTION * inches / HardwareTilerunner.WHEEL_CIRCUMFERENCE);

        tileRunner.motorPair.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tileRunner.motorPair.setTargetPosition( ticks );
        tileRunner.motorPair.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        tileRunner.motorPair.setPower(power);

        while (tileRunner.leftMotor.isBusy() && waitHandler.isActive()) {}
    }

    static void stop(HardwareTilerunner tileRunner) {
        tileRunner.motorPair.setPower(0);
    }

    static void turn(HardwareTilerunner tileRunner, BusyWaitHandler waitHandler, double direction, double degrees) {
        double heading = tileRunner.getHeading();

        tileRunner.motorPair.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tileRunner.leftMotor.setPower(direction);
        tileRunner.rightMotor.setPower(-direction);

        while ( Math.abs(heading - tileRunner.getHeading()) < degrees && waitHandler.isActive()) {}
        tileRunner.motorPair.setPower(0);
    }
}
