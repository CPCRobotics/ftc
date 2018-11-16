package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Autonomous.Sampling;
import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;


@Autonomous(name="NullAuto", group="Test")
public class NullAuto extends LinearOpMode
{
	@Override
	public void runOpMode()
	{
		telemetry.addData("NullAuto", "Initializing");
		telemetry.update();

		// Initialize the OpModeKeeper Singleton so other parts of our code can use it to get a reference to the OpMode object.
		OpModeKeeper.setOpMode( this );

		// Setup the resources needed by the Sampling class and create it.
		String vuforiaKey =  hardwareMap.appContext.getString(R.string.vuphoriaLicense);
		int viewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		WebcamName camera = hardwareMap.get(WebcamName.class, "webcam");
		Sampling sampling = new Sampling( null, telemetry, vuforiaKey, viewId, camera );

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

		telemetry.addData("NullAuto", "Starting");
		telemetry.update();


		// Spin here updating telemetry until OpMode terminates
		while ( opModeIsActive() )
		{
			telemetry.update();
		}
	}
}
