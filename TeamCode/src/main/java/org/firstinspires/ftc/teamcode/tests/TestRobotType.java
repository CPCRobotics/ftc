package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.EasyLinearOpmode;
import org.firstinspires.ftc.teamcode.twigger.Twigger;
import org.firstinspires.ftc.teamcode.util.RobotType;

/**
 * @see org.firstinspires.ftc.teamcode.util.RobotType
 */
@Autonomous(name="Test Robot Type", group="TestSoftware")
public class TestRobotType extends EasyLinearOpmode {
    @Override
    protected void startAutonomous() throws InterruptedException {
        Twigger.getInstance()
                .addData("type", RobotType.detect(hardwareMap))
                .update();


        // Wait forever to read telemetry; finishing the opmode will
        // remove all text
        while (opModeIsActive())
            Thread.sleep(50);
    }
}
