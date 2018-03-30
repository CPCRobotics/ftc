package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.AutonomousOpmode;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Report limit switch outputs
 */
@TeleOp
public class TestLimitSwitch extends AutonomousOpmode {
    @Override
    protected void startAutonomous() throws InterruptedException {
        while (true) {
            Twigger.getInstance()
                    .addData("top", tilerunner.isLiftAtHighPoint())
                    .addData("bottom", tilerunner.isLiftAtLowPoint())
                    .update();

            Thread.sleep(500);
        }
    }
}
