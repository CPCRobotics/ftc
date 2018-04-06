package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.EasyLinearOpmode;

/**
 * Move an arbitrarily long distance forward
 */
@Autonomous(name="Test Perpetual Move", group="TestConcept")
public class TestPerpetualMove extends EasyLinearOpmode {
    @Override
    protected void startAutonomous() throws InterruptedException {
        final double SPEED = 0.3;

        tilerunner.move(this, SPEED, 9999999);
    }
}
