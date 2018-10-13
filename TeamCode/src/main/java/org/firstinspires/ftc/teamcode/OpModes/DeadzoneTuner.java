package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TileRunner;
import org.firstinspires.ftc.teamcode.Util.IMUSensor;
import org.firstinspires.ftc.teamcode.Util.NavUtils;
import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;

@Autonomous(name="Deadzone Tuner", group="Test")
public class DeadzoneTuner extends LinearOpMode {

    /* Declare OpMode members. */
    TileRunner         robot   = new TileRunner();

    @Override
    public void runOpMode() throws InterruptedException
	{
		telemetry.addData("Initializing", "Initializing");
		telemetry.update();
		robot.init( hardwareMap );

		// Initialize the OpModeKeeper Singleton so other parts of our code can use it to get a reference to the OpMode object.
		OpModeKeeper.setOpMode( this );

		// Create an IMUSensor object using the IMU on the robot.
		IMUSensor imu = new IMUSensor( robot.imu );

		// Create the NavUtils object that we will use to drive the robot.
		NavUtils nav = new NavUtils( robot.leftDrive, robot.rightDrive, imu, 4.0, telemetry );

		// Pause here waiting for the Run button on the driver station to be pressed.
		waitForStart();
		telemetry.addLine("Running");

		double currentPower = 0.02;
		int direction = 1;
		while(true && opModeIsActive())
		{
			telemetry.addData("Power", "" + currentPower);
			telemetry.update();
			robot.leftDrive.setPower(currentPower * direction);
			robot.rightDrive.setPower(currentPower * direction);
			sleep(4000);
			robot.leftDrive.setPower( 0 );
			robot.rightDrive.setPower( 0 );
			sleep(2000);
			currentPower += 0.02;
			direction *= -1;
		}

		// Stop all motors
		robot.leftDrive.setPower( 0 );
		robot.rightDrive.setPower( 0 );
    }
}
