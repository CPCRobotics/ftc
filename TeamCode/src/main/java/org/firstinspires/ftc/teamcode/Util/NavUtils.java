package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class NavUtils {
	DcMotor		leftMotor = null;
	DcMotor 	rightMotor = null;
	IMUSensor	imu = null;
	int			ticksPerInch = 0;
	private		PIDController turnPID = null;
	private 	Telemetry telemetry;

	// Constants
	private final double TURN_TARGET_THRESHOLD = 2;
	private final double TURN_POWER = 0.5;

	public NavUtils( DcMotor left, DcMotor right, IMUSensor imu, double wheelDiameter, Telemetry tel )
	{
		telemetry = tel;
		leftMotor = left;
		rightMotor = right;
		this.imu = imu;

		//testing telemetry
		telemetry.addLine("Testing nav util telemetry");
		telemetry.addLine("Testing...");
		telemetry.update();

		// Get the number of encoder ticks per revolution from the motor configuration.
		double motorTicksPerRevolution = left.getMotorType().getTicksPerRev();

		// Calculate circumference of robots drive wheels
		double circumference = wheelDiameter * Math.PI;

		// Calculate motor ticks per inch of movement
		ticksPerInch = (int)( motorTicksPerRevolution / circumference );

		// Create the PID controller used by our IMU turn code.  These PID constants were
		// determined experimentally during the 2017-2018 season, we might want to improve them.
		turnPID = new PIDController( TURN_POWER, 0.04, 0.0001, 0.005 );
	}

	/**
	 * Drive the robot straight forwards or backwards a given distance.
	 *
	 * @param distance - Distance to drive in inches, negative values drive backwards.
	 */
	public void drive( double distance )
	{
		// First ensure that the motors are stopped and reset the encoders to zero.
		leftMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		rightMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		// Calculate the number of motor ticks we need to rotate to move robot desired distance.
		int ticks = (int)(distance * ticksPerInch);

		leftMotor.setTargetPosition( ticks );
		rightMotor.setTargetPosition( ticks );

		// In RUN_TO_POSITION mode the motor controller will use an internal PID controller
		// to attempt to move the motor to the specified position and hold it there.
		leftMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
		rightMotor.setMode( DcMotor.RunMode.RUN_TO_POSITION );

		// Just spin here until the motor has finished running or the OpMode is told to exit.
		while (leftMotor.isBusy() && OpModeKeeper.isActive())
		{
		}
	}


	public void turn( double angle )
			throws InterruptedException
	{
		turn( angle, 0 );
	}


	public void turn( double angle, double maxSecs)
			throws InterruptedException
	{
	    telemetry.addData("Starting turn", "Angle: " + angle);
	    telemetry.update();
		double currentHeading;
		double targetHeading;
		double error;

		final ElapsedTime timeoutTimer = new ElapsedTime();
		final ElapsedTime momentumTimer = new ElapsedTime();

		// Turn algorithm uses positive angles to turn CCW. .turn() promises positive angles turns CW instead.
		angle *= -1;

		currentHeading = imu.getHeading();
		targetHeading = currentHeading + angle;

		leftMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		rightMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

		while (OpModeKeeper.isActive() && (maxSecs <= 0 || timeoutTimer.seconds() < maxSecs))
		{
			// Calculate the error (delta between the current and target headings) then use
			// the PID controller to determine the correct motor power to use.
			error = targetHeading - imu.getHeading();
			double currentPower = turnPID.get(error);
			leftMotor.setPower(-currentPower);
			rightMotor.setPower(currentPower);

			//telemetry for testing
            telemetry.addData("Max Seconds", "" + maxSecs);
            telemetry.addData("timeoutTimer", "" + timeoutTimer.seconds());
            telemetry.addData("current power", "" + currentPower);
            telemetry.addData("Error", "" + error);
            telemetry.addData("Heading", "" + imu.getHeading());
            telemetry.update();

			// If the current error has been within our target range for long enough
			// then we are done so break out of the loop.
			if (Math.abs(error) > TURN_TARGET_THRESHOLD ) {
                momentumTimer.reset();
			} else if (momentumTimer.seconds() > 0.1) {
                break;
			}

			Thread.sleep(10);
		}

		leftMotor.setPower(0);
		rightMotor.setPower( 0 );

		leftMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		rightMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		Thread.sleep(100); // Give robot a moment to come to a stop.
	}
}
