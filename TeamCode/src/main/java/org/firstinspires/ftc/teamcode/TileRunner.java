package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mocks.DcMotorMock;

/**
 * Hardware class for a basic org.firstinspires.ftc.teamcode.TileRunner robot with a Rev Expansion hub.
 * <p>
 * Moving the hardware specific code into a separate class allow all of our OpModes to
 * share it and reduced duplicate code.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left drive motor:        "left_drive"
 * Motor channel:  Right drive motor:       "right_drive"
 * Sensor: Bosch BNO055 IMU                 "imu"
 */
public class TileRunner {
	/* Public OpMode members. */
	public DcMotor leftDrive = null;
	public DcMotor rightDrive = null;
	public BNO055IMU imu = null;

	/* local OpMode members. */
	HardwareMap hardwareMap = null;

	/* Constructor */
	public TileRunner() {

	}

	/* Initialize standard Hardware interfaces */
	public void init( HardwareMap hwMap ) {

		// Save reference to Hardware Map
		hardwareMap = hwMap;

		// Define and Initialize Motors
		leftDrive = GetDcMotor( "left_drive" );
		rightDrive = GetDcMotor( "right_drive" );

		// Make sure motors are initially stopped
		leftDrive.setPower( 0 );
		rightDrive.setPower( 0 );

		// Set motor rotation direction for positive power values.  AndyMark motors are opposite
		// of Tetrix motors so this will need to be changed for different motor types.
		leftDrive.setDirection( DcMotor.Direction.REVERSE );
		rightDrive.setDirection( DcMotor.Direction.FORWARD );

		// Set all motors to run without using position encoders.
		leftDrive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		rightDrive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

		// Get and initialize IMU
		// imu = hardwareMap.get( BNO055IMU.class, "imu" );
	}


	public DcMotor GetDcMotor( String motorName )
	{
		DcMotor motor = null;
		try {
			motor = hardwareMap.get( DcMotor.class, motorName );
		}
		catch (Exception ex )
		{
			motor = new DcMotorMock();
		}

		return motor;
	}
}

