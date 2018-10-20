package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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
	public DcMotor lift = null;
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
		lift = GetDcMotor( "lift" );

		// Make sure motors are initially stopped
		leftDrive.setPower( 0 );
		rightDrive.setPower( 0 );
		lift.setPower(0);

		// Set motor rotation direction for positive power values.  AndyMark motors are opposite
		// of Tetrix motors so this will need to be changed for different motor types.
		leftDrive.setDirection( DcMotor.Direction.FORWARD );
		rightDrive.setDirection( DcMotor.Direction.REVERSE );
		lift.setDirection(DcMotor.Direction.REVERSE);

		// Set all motors to run without using position encoders.
		leftDrive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		rightDrive.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		lift.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

		// Get and initialize IMU
		imu = hardwareMap.get( BNO055IMU.class, "imu" );

		// Set up the parameters with which we will use our IMU. Note that integration
		// algorithm here just reports accelerations to the logcat log; it doesn't actually
		// provide positional information.
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.calibrationDataFile  = "RevHubIMUCalibration.json";
		parameters.loggingEnabled       = true;
		parameters.loggingTag           = "IMU";
		parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

		imu.initialize( parameters );
	}


	public DcMotor GetDcMotor( String motorName )
	{
		DcMotor motor = null;
		try {
			motor = hardwareMap.get( DcMotor.class, motorName );
		}
		catch (Exception ex )
		{
//			motor = new DcMotorMock();
		}

		return motor;
	}
}

