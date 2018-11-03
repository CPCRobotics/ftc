package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.R;
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
    public void runOpMode() throws InterruptedException
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

        //create the mineral detector
        String vuforiaKey =  hardwareMap.appContext.getString(R.string.vuphoriaLicense);
        int viewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        WebcamName camera = hardwareMap.get(WebcamName.class, "webcam");

        //create sampling object
        Sampling sampling = new Sampling( nav, telemetry, vuforiaKey, viewId, null );

        //start locating the position of the minerals
        sampling.startRecognition();

        // Pause here waiting for the Run button on the driver station to be pressed.
        while (!isStarted())
        {
            sampling.locate();

            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }

        //stop locating minerals
        sampling.stopRecognition();

        telemetry.addData("CraterSide", "Starting");
        telemetry.update();

        // Call the set of strategies the will accomplish the tasks for this run of autonomous.
        Landing.Land( robot.lift, robot.liftUpperLimit, nav);

		sampling.Collect();

        DriveToDepot( nav );

        Claiming.DeployMarker( nav, robot.arm );

        Parking.ParkInCrater(nav);

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
