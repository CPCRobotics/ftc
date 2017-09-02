package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by samuel on 8/11/17.
 */

class TilerunnerUtils {
    private TilerunnerUtils() {}
    private static double THRESHOLD_TICKS = HardwareTilerunner.TICKS_PER_REVOLUTION;
    private static double THRESHOLD_HEADING = 30;

    private static double difference(double source, double now) {
        return Math.abs(source - now) % 360;
    }

    /**
     *
     * @return the speed to run according to the goal distance left
     */
    private static double calculateSpeed(double dist, double threshold) {

        // Slow down when the distance to the target is less than 3 * the distance of the wheel
        return Math.min(1, Math.max(0.5, dist / threshold) );
    }

    static void move(HardwareTilerunner tileRunner, BusyWaitHandler waitHandler, double inches, double power) {
        int ticks = (int)(HardwareTilerunner.TICKS_PER_REVOLUTION * inches / HardwareTilerunner.WHEEL_CIRCUMFERENCE);

        DcMotor motorPair = tileRunner.motorPair;

        motorPair.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPair.setTargetPosition( ticks );
        motorPair.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorPair.setPower(power);

        while (tileRunner.leftMotor.isBusy() && waitHandler.isActive()) {
            motorPair.setPower(calculateSpeed(motorPair.getTargetPosition() - motorPair.getCurrentPosition(), THRESHOLD_TICKS));
        }
    }

    static void stop(HardwareTilerunner tileRunner) {
        tileRunner.motorPair.setPower(0);
    }

    static void turn(HardwareTilerunner tileRunner, BusyWaitHandler waitHandler, double direction, double degrees) {
        double heading = tileRunner.getHeading();
        DcMotor motorPair = tileRunner.motorPair;

        motorPair.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tileRunner.leftMotor.setPower(direction);
        tileRunner.rightMotor.setPower(-direction);

        while ( difference(heading, tileRunner.getHeading()) > degrees && waitHandler.isActive()) {
            double power = direction * calculateSpeed(difference(heading, tileRunner.getHeading()), THRESHOLD_HEADING);
            tileRunner.telemetry.addData("speed", power);
            tileRunner.telemetry.addData("delta", difference(heading, tileRunner.getHeading()));
            tileRunner.leftMotor.setPower(power);
            tileRunner.rightMotor.setPower(-power);
        }
        tileRunner.motorPair.setPower(0);
    }
}
