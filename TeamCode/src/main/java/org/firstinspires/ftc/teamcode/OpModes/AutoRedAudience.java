package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TileRunner;
import org.firstinspires.ftc.teamcode.Util.IMUSensor;
import org.firstinspires.ftc.teamcode.Util.NavUtils;
import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;

@Autonomous(name= "AutoRedAudience", group="Competition")
public class AutoRedAudience extends LinearOpMode {

    /* Declare OpMode members. */
    TileRunner         robot   = new TileRunner();

    @Override
    public void runOpMode()
			throws InterruptedException
	{
		telemetry.addLine("Initializing");

		robot.init( hardwareMap );

		// Initialize the OpModeKeeper Singleton so other parts of our code can use it to get a reference to the OpMode object.
		OpModeKeeper.setOpMode( this );

		// Create an IMUSensor object using the IMU on the robot.
		IMUSensor imu = new IMUSensor( robot.imu );

		// Create the NavUtils object that we will use to drive the robot.
		NavUtils nav = new NavUtils( robot.leftDrive, robot.rightDrive, imu, 4.0, telemetry );

		// Pause here waiting for the Run button on the driver station to be pressed.
		waitForStart();

		//move to
		telemetry.addLine("moving to minerals");
		nav.drive(13, 1);
		telemetry.addLine("turning to wall");
		nav.samTurn(1, -70);
		telemetry.addLine("moving to wall");
		nav.drive(35, 1);
		telemetry.addLine("turning to depot");
		nav.samTurn(1,-64);
		telemetry.addLine("moving to depot");
		nav.drive(35, 1);
		//nav.rotate(180);
		nav.drive(-76, 1);

		while ( opModeIsActive() ) {
			telemetry.update();
		}

		// Stop all motors
		robot.leftDrive.setPower( 0 );
		robot.rightDrive.setPower( 0 );
    }
}
