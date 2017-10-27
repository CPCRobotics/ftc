package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.strategy.AutonomousStrategy;
import org.firstinspires.ftc.teamcode.strategy.TeamPosition;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Created by samuel on 10/27/17.
 */

@Autonomous(name="Auto Red Audience", group="Competition")
public class AutoRedAudience extends LinearOpMode implements BusyWaitHandler {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousStrategy strategy = new AutonomousStrategy(TeamPosition.RED_A, this,
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
