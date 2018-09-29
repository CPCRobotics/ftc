package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.EasyLinearOpmode;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Displays the top elevator position after it reaches the hardware limit switch
 */
@Autonomous(name="Test Elevator Top", group="TestHardware")
public class TestGetTopPosition extends EasyLinearOpmode {
    @Override
    protected void startAutonomous() throws InterruptedException {
        // NOTE: run RestLift before this opmode because this doesn't work for an unknown reason
        tilerunner.zeroLift(this);


        tilerunner.setIgnoreSoftwareLimits(true);
        while (!tilerunner.isLiftAtHighPoint())
            tilerunner.setLiftPower(0.15);

        Twigger.getInstance().sendOnce("ELEVATOR POSITION: " + tilerunner.getElevatorPos()).update();
        Thread.sleep(10000);
    }
}
