package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;
import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.util.PIDTuner;

/**
 * Drives the robot back and forth to test Tilerunner.move().
 *
 * For PID Tuning
 */
@TeleOp(name="Tune PID .move()", group="Tune")
public class TunePIDMove extends LinearOpMode implements BusyWaitHandler {
    private PIDTuner tuner;

        @Override
    public void runOpMode() throws InterruptedException {
        final double DEST_IN = 36;

        Tilerunner tilerunner = new Tilerunner();
        tilerunner.init(hardwareMap, telemetry, Tilerunner.OpmodeType.AUTONOMOUS);
        tuner = new PIDTuner(RobotMap.PID_MOVE, 1, gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            tilerunner.move(this, DEST_IN, tuner.get(), 10);
            longSleep();
            tilerunner.move(this, -DEST_IN, tuner.get(), 10);
            longSleep();
        }
    }

    private void longSleep() throws InterruptedException {
        final ElapsedTime timer = new ElapsedTime();
        while (isActive() && timer.milliseconds() < 1000) {
            Thread.sleep(50);
        }
    }

    @Override
    public boolean isActive() {
        tuner.update();
        Twigger.getInstance().update();
        return opModeIsActive();
    }
}
