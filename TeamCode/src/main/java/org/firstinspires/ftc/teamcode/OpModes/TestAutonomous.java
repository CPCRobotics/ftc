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


@Autonomous(name="TestAutonomous", group="Test")
public class TestAutonomous extends LinearOpMode
{
	@Override
	public void runOpMode() throws InterruptedException
	{
		telemetry.addData("TestAutonomous", "Initializing");
		telemetry.update();

		// Initialize the OpModeKeeper Singleton so other parts of our code can use it to get a reference to the OpMode object.
		OpModeKeeper.setOpMode( this );

		//create the mineral detector
		String vuforiaKey =  hardwareMap.appContext.getString(R.string.vuphoriaLicense);
		int viewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//		WebcamName camera = hardwareMap.get(WebcamName.class, "webcam");

		//create sampling object
		Sampling sampling = new Sampling( null, telemetry, vuforiaKey, viewId, null );

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

		telemetry.addData("TestAutonomous", "Starting");
		telemetry.update();


		// Spin here updating telemetry until OpMode terminates
		while ( opModeIsActive() )
		{
			telemetry.update();
		}
	}
}
