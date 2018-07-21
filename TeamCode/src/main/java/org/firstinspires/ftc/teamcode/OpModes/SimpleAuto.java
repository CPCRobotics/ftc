package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TileRunner;

@Autonomous( name = "Simple", group = "Competition" )
public class SimpleAuto extends LinearOpMode
{

	/* Declare OpMode members. */
	TileRunner robot = new TileRunner();

	@Override
	public void runOpMode()
	{

		/* Initialize the drive system variables.
		 * The init() method of the hardware class does all the work here
		 */
		robot.init( hardwareMap );

		// If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
		//robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		//robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		// Send telemetry message to signify robot waiting;
		telemetry.addData( "Status", "Ready to run" );    //
		telemetry.update();

		// Wait for the game to start (driver presses PLAY)
		// Abort this loop is started or stopped.
		while ( !( isStarted() || isStopRequested() ))
		{

			// Display the light level while we are waiting to start
			telemetry.addData( "Fnord", 0 );
			telemetry.update();
			idle();
		}

		// Start the robot moving forward, and then begin looking for a white line.
		robot.leftDrive.setPower( 0.25 );
		robot.rightDrive.setPower( 0.25 );
	}
}

