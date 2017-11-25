package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.strategy.AutonomousStrategy;
import org.firstinspires.ftc.teamcode.strategy.TeamPosition;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Brings the lift down and resets the encoder.
 */
@Autonomous(name="Reset Lift", group="Competition")
public class ResetLift extends LinearOpMode implements BusyWaitHandler {

    @Override
    public void runOpMode() throws InterruptedException {
        Tilerunner tilerunner = new Tilerunner();
        tilerunner.init(hardwareMap, telemetry);

        waitForStart();

        tilerunner.zeroLift(this);
    }

    @Override
    public boolean isActive() {
        Twigger.getInstance().update();
        return opModeIsActive();
    }
}
