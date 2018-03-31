package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AutonomousOpmode;

/**
 * Drives the robot back and forth to test Tilerunner.move()
 */
@Autonomous(name="Test Move", group="TestConcept")
public class TestMove extends AutonomousOpmode {
    @Override
    protected void startAutonomous() throws InterruptedException {
        final double POWER = 0.5;
        final double DEST_IN = 36;

        while (opModeIsActive()) {
            tilerunner.move(this, POWER, DEST_IN);
            Thread.sleep(1000);
            tilerunner.move(this, POWER, -DEST_IN);
            Thread.sleep(1000);
        }
    }
}
