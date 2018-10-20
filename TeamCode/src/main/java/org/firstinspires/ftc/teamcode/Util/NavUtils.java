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
	private		VirtualCompass compass;

	// Constants
	private final double TURN_TARGET_THRESHOLD = 0.5;
	private final double TURN_POWER = 0.5;

	public static final double MOTOR_DEADZONE = 0.05; // range [0,1]

	public final double MIN_TURN_POWER = 0.2;
	public final double ADJUST_THRESHHOLD = 2;

	public NavUtils( DcMotor left, DcMotor right, IMUSensor imu, double wheelDiameter, Telemetry tel )
	{
		telemetry = tel;
		leftMotor = left;
		rightMotor = right;
		this.imu = imu;
		compass = new VirtualCompass(imu);

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
	public void drive( double distance, double power )
	{
		rightMotor.setPower(-power);
		leftMotor.setPower(power);

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


	public void turn( double angle, double maxSecs) throws InterruptedException
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

		currentHeading = compass.getAngle();
		targetHeading = currentHeading + angle;

		leftMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		rightMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		leftMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		rightMotor.setMode( DcMotor.RunMode.RUN_USING_ENCODER );

		int sleeptime = 10;

		while (OpModeKeeper.isActive() && (maxSecs <= 0 || timeoutTimer.seconds() < maxSecs))
		{
			// Calculate the error (delta between the current and target headings) then use
			// the PID controller to determine the correct motor power to use.
			error = targetHeading - compass.getAngle();

			// If the current error has been within our target range for long enough
			// then we are done so break out of the loop.
			if (Math.abs(error) > TURN_TARGET_THRESHOLD ) {
				momentumTimer.reset();
			} else if (momentumTimer.seconds() > 0.1) {
				break;
			}

			double currentPower = turnPID.get(error);


			if(currentPower * error < 0)
			{
				currentPower *= -1;
			}

			if(error < 10)
			{
				sleeptime = 1;
			}

			if(Math.abs(error) < 4)
			{
				//sleeptime = 1;
				currentPower = MOTOR_DEADZONE * Math.signum(currentPower);
			}

			// ensure movement is powerful enough
			currentPower = Math.max(MOTOR_DEADZONE, Math.abs(currentPower)) * Math.signum(currentPower);
			leftMotor.setPower(-currentPower);
			rightMotor.setPower(currentPower);

			//telemetry for testing
            telemetry.addData("Max Seconds", "" + maxSecs);
            telemetry.addData("timeoutTimer", "" + timeoutTimer.seconds());
            telemetry.addData("current power", "" + currentPower);
            telemetry.addData("Error", "" + error);
            telemetry.addData("Heading", "" + imu.getHeading());
            telemetry.update();

			Thread.sleep(sleeptime);
		}

		leftMotor.setPower(0);
		rightMotor.setPower( 0 );

		leftMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
		rightMotor.setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

		Thread.sleep(100); // Give robot a moment to come to a stop.
	}

	public void rotate(double degrees) throws InterruptedException
	{
		degrees *= -1;

		leftMotor.setPower(0);
		rightMotor.setPower(0);

		double startingAngle = compass.getAngle();
		double targetAngle = startingAngle + degrees;
		double turnPower;
		double currentAngle;
		double degreesFrom;
		int direction;
		double previousError = 1000;
		//this timer is used to measure how long the robot is at the position it needs to be at
		final ElapsedTime momentumTimer = new ElapsedTime();

		//Rough Adjustment
		while(OpModeKeeper.isActive())
		{
			currentAngle = compass.getAngle();
			direction = (int)Math.signum(targetAngle - currentAngle);
			degreesFrom = Math.abs(targetAngle - currentAngle);

			//Math.abs(previousError - currentAngle) < 2
			//Alternate formula Math.abs(degreesFrom) < 15
			turnPower = degreesFrom / 100 * direction;
			if(Math.abs(degreesFrom) < 20 || Math.abs(turnPower) < MIN_TURN_POWER)
			{
				turnPower = MIN_TURN_POWER * direction;
			}

			telemetry.addData("direction", "" + direction);
			telemetry.addData("degrees from", "" + degreesFrom);
			telemetry.addData("turnPower", "" + turnPower);
			telemetry.update();


			leftMotor.setPower(-turnPower);
			rightMotor.setPower(turnPower);

			if(degreesFrom < ADJUST_THRESHHOLD && degreesFrom > -ADJUST_THRESHHOLD && momentumTimer.seconds() >= 0.2)
			{
				telemetry.addLine("Finished turn");
				leftMotor.setPower(0);
				rightMotor.setPower(0);
				break;
			}
			else if(degreesFrom < ADJUST_THRESHHOLD && degreesFrom > -ADJUST_THRESHHOLD)
			{

			}
			else
			{
				momentumTimer.reset();
			}
			//testing for pulse turning
//			Thread.sleep(100);
//			leftMotor.setPower(0);
//			rightMotor.setPower(0);
			previousError = degreesFrom;
		}
		Thread.sleep(500);
	}

	//testing new driving method
	public void move(int distance) throws InterruptedException
	{
		leftMotor.setPower(0);
		rightMotor.setPower(0);

		int startingTicks = leftMotor.getCurrentPosition();
		int targetTicks = startingTicks + (int)(distance * ticksPerInch);
		double turnPower;
		int currentPosition;
		double ticksFrom;
		int direction;
		double previousError = 1000;
		//this timer is used to measure how long the robot is at the position it needs to be at
		final ElapsedTime momentumTimer = new ElapsedTime();

		//Rough Adjustment
		while(OpModeKeeper.isActive())
		{
			currentPosition = leftMotor.getCurrentPosition();
			direction = (int)Math.signum(targetTicks - currentPosition);
			ticksFrom = Math.abs(targetTicks - currentPosition);

			//Math.abs(previousError - currentAngle) < 2
			//Alternate formula Math.abs(degreesFrom) < 15
			turnPower = ticksFrom / 2500 * direction;
			if(Math.abs(turnPower) < 0.15)
			{
				turnPower = 0.1 * direction;
			}

			telemetry.addData("direction", "" + direction);
			telemetry.addData("ticks from", "" + ticksFrom);
			telemetry.addData("turnPower", "" + turnPower);
			telemetry.update();

			leftMotor.setPower(turnPower);
			rightMotor.setPower(turnPower);

			if(ticksFrom < 10 && ticksFrom > -10 && momentumTimer.seconds() >= 0.2)
			{
				telemetry.addLine("Finished move");
				leftMotor.setPower(0);
				rightMotor.setPower(0);
				break;
			}
			else if(ticksFrom < 10 && ticksFrom > -10)
			{

			}
			else
			{
				momentumTimer.reset();
			}
			previousError = ticksFrom;
		}
		Thread.sleep(500);
	}
}
