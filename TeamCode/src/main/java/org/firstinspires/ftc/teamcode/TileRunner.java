package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;


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
	public DcMotor arm = null;
	public Servo dumper = null;
	public DcMotor intake = null;
	public BNO055IMU imu = null;
	public DigitalChannel liftUpperLimit = null;
	public DigitalChannel liftLowerLimit = null;

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
		arm = GetDcMotor("arm");
		dumper = hardwareMap.get(Servo.class, "dumper");
		intake = hardwareMap.get(DcMotor.class, "intake");
		liftUpperLimit = hardwareMap.get(DigitalChannel.class, "lift_upper_limit");
		liftLowerLimit = hardwareMap.get(DigitalChannel.class, "lift_lower_limit");

		//tells certain motors to brake when power is zero
		arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


		// Make sure motors are initially stopped
		leftDrive.setPower(0);
		rightDrive.setPower(0);
		lift.setPower(0);
		arm.setPower(0);
		intake.setPower(0);

		// Set motor rotation direction for positive power values.  AndyMark motors are opposite
		// of Tetrix motors so this will need to be changed for different motor types.
		leftDrive.setDirection( DcMotor.Direction.REVERSE);
		rightDrive.setDirection( DcMotor.Direction.FORWARD);
		lift.setDirection(DcMotor.Direction.FORWARD);
		arm.setDirection(DcMotor.Direction.FORWARD);
		intake.setDirection(DcMotor.Direction.FORWARD);

		// Set all motors to run without using position encoders.
		leftDrive.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		rightDrive.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
		lift.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
		arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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


	/**
	 * The idea here is to allow us to return a mock motor class when running on a robot
	 * that does not have all the same motors configured that the competition robot does.
	 * This way we can run all the same code on the software robot which might not have
	 * all motors the competition robot does.
	 *
	 * Currently this method just returns null if the named motor is not configured, it really should
	 * return a mock object that could be called by other code without generating an exception.
	 *
	 * @param motorName - Name of the motor to get from the hardware map.
	 *
	 * @return - DcMotor object for the name motor or null if no motor with that name is configured.
	 */
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
	public static void lowerLift(DcMotor motor, DigitalChannel LowerLimit)
	{
		//set motor power
		motor.setPower(-0.25);

		while(OpModeKeeper.isActive() && !LowerLimit.getState())
		{}

		motor.setPower(0);
	}
}

