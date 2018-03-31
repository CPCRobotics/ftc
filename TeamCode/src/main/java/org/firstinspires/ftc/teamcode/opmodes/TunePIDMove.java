package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        tuner = new PIDTuner(0.5, gamepad1);
        Tilerunner tilerunner = new Tilerunner();
        tilerunner.init(hardwareMap, telemetry, Tilerunner.OpmodeType.AUTONOMOUS);

        waitForStart();

        while (opModeIsActive()) {
            tilerunner.move(this, DEST_IN, tuner.get());
            longSleep();
            tilerunner.move(this, -DEST_IN, tuner.get());
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
