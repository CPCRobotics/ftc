package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.strategy.AutonomousStrategy;
import org.firstinspires.ftc.teamcode.strategy.TeamPosition;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Created by samuel on 10/14/17.
 */

@Autonomous(name="Reset Lift", group="Competition")
public class ResetLift extends LinearOpMode implements BusyWaitHandler {

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousStrategy strategy = new AutonomousStrategy(TeamPosition.RESET_LIFT,
                this, this);

        waitForStart();
    }

    @Override
    public boolean isActive() {
        Twigger.getInstance().update();
        return opModeIsActive();
    }
}
