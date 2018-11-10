package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TileRunner;
import org.firstinspires.ftc.teamcode.Autonomous.Landing;
import org.firstinspires.ftc.teamcode.Autonomous.Sampling;
import org.firstinspires.ftc.teamcode.Autonomous.Claiming;
import org.firstinspires.ftc.teamcode.Autonomous.Parking;
import org.firstinspires.ftc.teamcode.Util.IMUSensor;
import org.firstinspires.ftc.teamcode.Util.NavUtils;
import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;


@Autonomous(name="CraterSide", group="Competition")
public class CraterSide extends LinearOpMode
{
    /* Declare OpMode members. */
    TileRunner         robot   = new TileRunner();

    @Override
    public void runOpMode()
            throws InterruptedException
    {
        telemetry.addData("CraterSide", "Initializing");
        telemetry.update();
        robot.init( hardwareMap );

        // Initialize the OpModeKeeper Singleton so other parts of our code can use it to get a reference to the OpMode object.
        OpModeKeeper.setOpMode( this );

        // Create an IMUSensor object using the IMU on the robot.
        IMUSensor imu = new IMUSensor( robot.imu );

        // Create the NavUtils object that we will use to drive the robot.
		telemetry.addLine("Initializing NavUtils...");
        NavUtils nav = new NavUtils( robot.leftDrive, robot.rightDrive, imu, 4.0, telemetry );

        // Pause here waiting for the Run button on the driver station to be pressed.
        waitForStart();
        telemetry.addData("CraterSide", "Starting");
        telemetry.update();

        // Call the set of strategies the will accomplish the tasks for this run of autonomous.
        Landing.Land( robot.lift, robot.liftUpperLimit);

        //sampling the minerals
		Sampling.Collect( Sampling.Position.RIGHT, nav );

		//drive into posishion
        DriveToDepot( nav );

        //Seting marker in the depo
        Claiming.DeployMarker( nav, robot.arm );

        //Parking in the crator
        Parking.ParkInCrater(nav);

        //Reseting the Lift
        robot.lowerLift(robot.lift, robot.liftLowerLimit);

        /*
        //lower lift (for convenience while testing)
        robot.lift.setPower( -0.8 );
        while( OpModeKeeper.isActive() && !robot.liftLowerLimit.getState()) { }
        robot.lift.setPower( 0 );
        */


        // Spin here updating telemetry until OpMode terminates
        while ( opModeIsActive() )
        {
            telemetry.update();
        }

        // Make sure drive motors are stopped.
        robot.leftDrive.setPower( 0 );
        robot.rightDrive.setPower( 0 );
    }


	/**
	 * Drive from our post-Sampling position to our alliances depot so we can deploy our team marker.
	 */
	void DriveToDepot( NavUtils nav ) throws InterruptedException
	{
	    //drive to wall
	    nav.drive(39, 1);
	    //turn part way towards depot
        nav.samTurn(1, -45);
        //start moving towards depot
        nav.drive(22, 1);
        //finish turning towards depot
	    nav.samTurn(1, -10);
	    //finish driving towards depot
        nav.drive(12, 1);
	}
}
