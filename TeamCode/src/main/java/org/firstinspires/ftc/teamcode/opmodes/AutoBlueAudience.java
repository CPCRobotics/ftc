package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.strategy.AutonomousDelegator;
import org.firstinspires.ftc.teamcode.strategy.TeamPosition;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Autonomous Op-mode for Blue Alliance, side near audience
 */
@Autonomous(name="Auto Blue Audience", group="Competition")
public class AutoBlueAudience extends LinearOpMode implements BusyWaitHandler {

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousDelegator strategy = new AutonomousDelegator(TeamPosition.BLUE_A,
                this, this);

        waitForStart();

        strategy.start();
    }

    @Override
    public boolean isActive() {
        Twigger.getInstance().update();
        return opModeIsActive();
    }
}
