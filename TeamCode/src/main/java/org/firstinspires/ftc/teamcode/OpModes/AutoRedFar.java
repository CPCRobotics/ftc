package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousDelegator;
import org.firstinspires.ftc.teamcode.autonomous.TeamPosition;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Autonomous op-mode for Red alliance, side away from audience
 */
@Autonomous(name="Auto Red Far", group="Competition")
public class AutoRedFar extends LinearOpMode implements BusyWaitHandler {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousDelegator strategy = new AutonomousDelegator(TeamPosition.RED_FAR, this,
                this);

        waitForStart();

        strategy.start();
    }

    @Override
    public boolean isActive() {
        Twigger.getInstance().update();
        return opModeIsActive();
    }
}