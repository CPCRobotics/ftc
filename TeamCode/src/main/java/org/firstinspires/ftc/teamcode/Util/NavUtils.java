package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;

public class NavUtils {
	DcMotor		leftMotor = null;
	DcMotor 	rightMotor = null;
	IMUSensor	imu = null;
	int			ticksPerInch = 0;

	// Constants
	private static final double TURN_THRESHOLD_DEG = 2;


	public NavUtils( DcMotor left, DcMotor right, IMUSensor imu, double wheelDiameter )
	{
		leftMotor = left;
		rightMotor = right;
		this.imu = imu;

		// Get the number of encoder ticks per revolution from the motor configuration.
		double motorTicksPerRevolution = left.getMotorType().getTicksPerRev();

		// Calculate circumference of robots drive wheels
		double circumference = wheelDiameter * Math.PI;

		// Calculate motor ticks per inch of movement
		ticksPerInch = (int)( motorTicksPerRevolution / circumference );
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


	public void turn( double angle, PIDController pid, double maxSecs)
			throws InterruptedException
	{
		double currentPosition;
		double error;

		final ElapsedTime timeoutTimer = new ElapsedTime();
		final ElapsedTime momentumTimer = new ElapsedTime();

		// Turn algorithm uses positive angles to turn CCW. .turn() promises positive angles turns CW instead.
		angle *= -1;

		currentPosition = imu.getHeading();
		final double dest = currentPosition + angle;

		leftMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		rightMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

		while (OpModeKeeper.isActive() && (maxSecs <= 0 || timeoutTimer.seconds() < maxSecs))
		{
			error = dest - imu.getHeading();

			double currentPower = pid.get(error);

			leftMotor.setPower(-currentPower);
			rightMotor.setPower(currentPower);

			// Ensure that it's within threshold for longer than enough
			if (Math.abs(error) > TURN_THRESHOLD_DEG) {
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

		Thread.sleep(100); // Wait for momentum to finish
	}
}
