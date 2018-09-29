package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousDelegator;
import org.firstinspires.ftc.teamcode.autonomous.TeamPosition;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Autonomous op-mode for Blue alliance, side away from audience
 */
@Autonomous(name="Auto Blue Far", group="Competition")
public class AutoBlueFar extends LinearOpMode implements BusyWaitHandler {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousDelegator strategy = new AutonomousDelegator(TeamPosition.BLUE_FAR, this,
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
