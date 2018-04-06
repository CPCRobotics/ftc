package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.EasyLinearOpmode;
import org.firstinspires.ftc.teamcode.strategy.JewelTopplerPhase;
import org.firstinspires.ftc.teamcode.strategy.TeamPosition;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Tests the Jewel Toppler portion of the Autonomous Mode
 */
@Autonomous(name="Jewel Toppler Test", group="TestConcept")
public class TestJewelToppler extends EasyLinearOpmode {
    private final JewelTopplerPhase strategy = new JewelTopplerPhase(TeamPosition.RED_A, this,
            hardwareMap.appContext, tilerunner);

    @Override
    protected void startAutonomous() throws InterruptedException {

        strategy.toppleEnemyJewel();
    }

    @Override
    public boolean isActive() {
        Twigger.getInstance().update();
        return opModeIsActive();
    }
}
