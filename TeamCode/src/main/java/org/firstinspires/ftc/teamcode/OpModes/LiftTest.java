package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TileRunner;


/**
 * Basic iterative Teleop OpMode for a tile runner based robot.
 */

@TeleOp( name = "Lift Test", group = "Test" )
public class LiftTest extends OpMode {

	// Declare OpMode members.
	TileRunner robot = new TileRunner();


	// Code to run ONCE when the driver hits INIT
	@Override
	public void init()
	{
		/* Initialize the hardware variables.
		 * The init() method of the hardware class does all the work here
		 */
		robot.init( hardwareMap );

		// Send telemetry message to signify robot waiting;
		telemetry.addData( "Say", "Hello Driver" );    //
	}


	// Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
	@Override
	public void init_loop()
	{
	}


	// Code to run ONCE when the driver hits PLAY
	@Override
	public void start() {
	}


	// Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
	@Override
	public void loop()
	{
		// Read the Y values of both joysticks, return values are [-1,1]
		double left = gamepad1.left_stick_y;

		// Set motor power for each side of robot to match values read from joysticks
		robot.lift.setPower( left );

		// Read the current motor encoder values so we can write them to telemetry.
		int liftPosition = robot.lift.getCurrentPosition();

		// Send telemetry data to driver station.
		telemetry.addData("Lift Position", "" + liftPosition);
		telemetry.addData("Lift Power", "" + left);
		telemetry.update();
	}

	 // Code to run ONCE after the driver hits STOP
	@Override
	public void stop()
	{
	}
}
