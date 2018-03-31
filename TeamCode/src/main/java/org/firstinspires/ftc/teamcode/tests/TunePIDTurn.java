package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;
import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.util.PIDTuner;

/**
 * Tune the PID live via the gamepad buttons
 */
@TeleOp(name="Tune PID .turn()", group="Tune")
public class TunePIDTurn extends LinearOpMode implements BusyWaitHandler {
    private PIDTuner tuner;

    @Override
    public void runOpMode() throws InterruptedException {

        tuner = new PIDTuner(0.5, gamepad1);
        Tilerunner tilerunner = new Tilerunner();
        tilerunner.init(hardwareMap, telemetry, Tilerunner.OpmodeType.AUTONOMOUS);

        waitForStart();

        int direction = 1;
        while (opModeIsActive()) {
            tilerunner.turn(this, 360 * direction, tuner.get());
            longSleep();
            direction *= -1;
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
