package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TileRunner;
import org.firstinspires.ftc.teamcode.Util.IMUSensor;
import org.firstinspires.ftc.teamcode.Util.NavUtils;
import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;

@Autonomous(name="TestAuto", group="Test")
public class TestAuto extends LinearOpMode {

    /* Declare OpMode members. */
    TileRunner         robot   = new TileRunner();

    @Override
    public void runOpMode()
			throws InterruptedException
	{
		telemetry.addData("Initializing", "Initializing");
		telemetry.update();
		robot.init( hardwareMap );

		// Initialize the OpModeKeeper Singleton so other parts of our code can use it to get a reference to the OpMode object.
		OpModeKeeper.setOpMode( this );

		// Create an IMUSensor object using the IMU on the robot.
		IMUSensor imu = new IMUSensor( robot.imu );


		telemetry.addLine("About to init nav util");
		// Create the NavUtils object that we will use to drive the robot.
		NavUtils nav = new NavUtils( robot.leftDrive, robot.rightDrive, imu, 4.0, telemetry );

		// Pause here waiting for the Run button on the driver station to be pressed.
		waitForStart();
		telemetry.addData("Running", "Running");
		telemetry.update();

		//nav.drive( 18 );
		nav.samTurn( 0.5 , 90 );
		sleep(2000);
		nav.samTurn( 0.5, 180 );
		sleep(2000);
		nav.samTurn( 0.5, 270 );

		sleep(4000);

		nav.samTurn( 0.5, -90 );
		sleep(2000);
		nav.samTurn( 0.5, -180 );
		sleep(2000);
		nav.samTurn( 0.5, -270 );
		sleep(2000);


		//nav.turn( -90 );

		//nav.drive( 1000 );



		while ( opModeIsActive() ) {
			telemetry.update();
		}

		// Stop all motors
		robot.leftDrive.setPower( 0 );
		robot.rightDrive.setPower( 0 );
    }
}
