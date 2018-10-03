package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TileRunner;
import org.firstinspires.ftc.teamcode.Util.IMUSensor;
import org.firstinspires.ftc.teamcode.Util.NavUtils;
import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;

@Autonomous(name="TestAuto", group="Test")
public class TestAuto extends LinearOpMode {

    /* Declare OpMode members. */
    TileRunner         robot   = new TileRunner();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

		// Initialize the OpModeKeeper Singleton so other parts of our code can use it to get a reference to the OpMode object.
        OpModeKeeper.setOpMode( this );

        // Create an IMUSensor object using the IMU on the robot.
		IMUSensor imu = new IMUSensor( robot.imu );

        // Create the NavUtils object that we will use to drive the robot.
		NavUtils nav = new NavUtils( robot.leftDrive, robot.rightDrive, imu, 4.0 );

		nav.drive( 18 );
		nav.drive( -18 );

        while (opModeIsActive())
        {
            telemetry.update();
        }

        // Stop all motors
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
    }
}
