package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.TileRunner;

@Autonomous( name = "Eric_SimpleAuto", group = "Competition" )
public class SimpleAuto extends LinearOpMode
{
	/* Declare OpMode members. */
	TileRunner robot = new TileRunner();

	private double driveSpeed = 0.25;

	@Override
	public void runOpMode()
	{
		/* Initialize the drive system variables.
		 * The init() method of the hardware class does all the work here
		 */
		robot.init( hardwareMap );

		// Send telemetry message to signify robot waiting;
		telemetry.addData( "Status", "Ready to run" );
		telemetry.update();

		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		//Drive forwards then turn three times
		for(int i = 0; i < 3; i++)
		{
			drive(driveSpeed, 2500);
			turn(driveSpeed, 2000);
		}
	}

	/**
	 * @param speed: The spped between -1 and 1 that the robot will drive
	 * @param time: The time the robot will drive forwards
	 */
	private void drive(double speed, double time()
	{
		robot.leftDrive.setPower(speed);
		robot.rightDrive.setPower(speed);
		sleep(time);
		stopDriving();
	}

	/**
	 * @param turnSpeed: speed the robot will turn at between -1 and 1
	 * @param time: Time the robot will turn
	 */
	private void turn(double turnSpeed, double time)
	{
		robot.leftDrive.setPower(turnSpeed);
		robot.rightDrive.setPower(-turnSpeed);
		sleep(time);
		stopDriving();
	}

	//Stops the left and right drive motors
	private void stopDriving()
	{
		robot.leftDrive.setPower(0);
		robot.rightDrive.setPower(0);
	}
}

