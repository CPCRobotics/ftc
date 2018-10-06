package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TileRunner;


/**
 * Basic iterative Teleop OpMode for a tile runner based robot.
 */

@TeleOp( name = "Tank Drive", group = "Competition" )
public class TankDrive extends OpMode {

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
		double right = gamepad1.right_stick_y;

		// Set motor power for each side of robot to match values read from joysticks
		robot.leftDrive.setPower( left );
		robot.rightDrive.setPower( right );

		// Read the current motor encoder values so we can write them to telemetry.
		int leftPos = robot.leftDrive.getCurrentPosition();
		int rightPos = robot.rightDrive.getCurrentPosition();

		// Read current heading from IMU
		Orientation angles = robot.imu.getAngularOrientation( AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		float heading = angles.firstAngle;

		// Send telemetry data to driver station.
		telemetry.addData( "Left Motor (power/pos): ", String.format( "%.2f/%d", left, leftPos) );
		telemetry.addData( "Right Motor (power/pos): ", String.format( "%.2f/%d", right, rightPos) );
		telemetry.addData( "Heading: ", "%.2f", heading );
		telemetry.update();
	}

	 // Code to run ONCE after the driver hits STOP
	@Override
	public void stop()
	{
	}
}
