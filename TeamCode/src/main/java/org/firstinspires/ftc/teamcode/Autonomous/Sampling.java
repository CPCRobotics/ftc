package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Util.NavUtils;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;
import java.util.concurrent.ExecutionException;

public class Sampling
{
	private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
	private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
	private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
	private static final double WALL_TURN_DEGREES = -77;

	private NavUtils nav = null;
	private Telemetry telemetry = null;
	private VuforiaLocalizer vuforia = null;
	private TFObjectDetector tfod = null;
	private ElapsedTime timer = null;

	private int leftCount = 0;
	private int rightCount = 0;
	private int centerCount = 0;
	private int unknownCount = 0;

	public enum Position {
		NULL,
		UNKNOWN,
		LEFT,
		CENTER,
		RIGHT
	};

	public Sampling( NavUtils nav, Telemetry telemetry, String vuforiaKey, int tfodMonitorViewId )
	{
		this.nav = nav;
		this.telemetry = telemetry;

		timer = new ElapsedTime( ElapsedTime.Resolution.MILLISECONDS );

		// Create the VuforiaLocalizer that will be used to capture frames of video.
		initVuforia(vuforiaKey);

		if ( ClassFactory.getInstance().canCreateTFObjectDetector())
		{
			initTfod(tfodMonitorViewId);
		}
		else
		{
			telemetry.addData("Sorry!", "This device is not compatible with TFOD");
		}
	}
	public void startRecognition()
	{
		/** Activate Tensor Flow Object Detection. */
		if (tfod != null)
		{
			tfod.activate();
		}
	}

	public void stopRecognition()
	{
		if (tfod != null)
		{
			tfod.shutdown();
		}
	}

	public void locate()
	{
		// We only poll the object detector once per second.
		if ( timer.milliseconds() < 1000 )
		{
			return;
		}
		timer.reset();


		Position result = Sampling.Position.NULL;

		if ( tfod != null )
		{
			// Get list of reognized objects.
			List<Recognition> recognitions = tfod.getRecognitions();
			int objectCount = recognitions.size();

			telemetry.addData( "Objects Detected", objectCount );

			if (objectCount == 3 )
			{
				int goldMineralY = -1;
				int silverMineral1Y = -1;
				int silverMineral2Y = -1;

				for (Recognition recognition : recognitions)
				{
					if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
					{
						goldMineralY = (int) recognition.getTop();
					}
					else if ( recognition.getLabel().equals(LABEL_SILVER_MINERAL) && silverMineral1Y == -1 )
					{
						silverMineral1Y = (int) recognition.getTop();
					}
					else if ( recognition.getLabel().equals(LABEL_SILVER_MINERAL) && silverMineral2Y == -1 )
					{
						silverMineral2Y = (int) recognition.getTop();
					}
					else
					{
						telemetry.addLine( "****** Unknown object ****" );
					}
				}

				// NOTE: The left and right are swapped because of our camera position
				if (goldMineralY != -1 && silverMineral1Y != -1 && silverMineral2Y != -1)
				{
					if (goldMineralY < silverMineral1Y && goldMineralY < silverMineral2Y)
					{
						result = Sampling.Position.LEFT;
						leftCount++;
					}
					else if (goldMineralY > silverMineral1Y && goldMineralY > silverMineral2Y)
					{
						result = Sampling.Position.RIGHT;
						rightCount++;
					}
					else if((goldMineralY > silverMineral1Y && goldMineralY < silverMineral2Y) || (goldMineralY < silverMineral1Y && goldMineralY > silverMineral2Y))
					{
						result = Sampling.Position.CENTER;
						centerCount++;
					}
					else
					{
						result = Sampling.Position.UNKNOWN;
						unknownCount++;
					}
				}

				telemetry.addData("Gold X", goldMineralY);
				telemetry.addData("Silver 1X", silverMineral1Y);
				telemetry.addData("Silver 2X", silverMineral2Y);
			}

		}

		// Display updated counts in telemetry.
		telemetry.addData("Left Count", leftCount);
		telemetry.addData("Center Count", centerCount);
		telemetry.addData("Right Count", rightCount);
		telemetry.addData("Unknown Count", unknownCount);
		telemetry.update();
	}

	public void Collect() throws InterruptedException
	{
		if(centerCount > leftCount && centerCount > rightCount)
		{
			CollectCenter();
		}
		else if(leftCount > rightCount)
		{
			CollectLeft();
		}
		else
		{
			CollectRight();
		}
	}


	private void CollectLeft() throws InterruptedException
	{
		nav.samTurn(1, -37.5);
		nav.drive(19, 0.7);
		nav.drive(-19, 0.7);
		//turn towards wall
		nav.samTurn(1, WALL_TURN_DEGREES + 37.5);
	}


	private void CollectCenter() throws InterruptedException
	{
		nav.drive(13, 0.7);
		nav.drive(-13, 0.7);
		//turn towards wall
		nav.samTurn(1, WALL_TURN_DEGREES);
	}


	private void CollectRight() throws InterruptedException
	{
		nav.samTurn(1, 37.5);
		nav.drive(19, 0.7);
		nav.drive(-19, 0.7);
		//turn towards wall
		nav.samTurn(1, -37.5 + WALL_TURN_DEGREES);
	}


	/**
	 * Initialize the Vuforia localization engine.
	 */
	private void initVuforia(String vuforiaKey)
	{
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 */
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		//parameters.vuforiaLicenseKey = vuphoriaLicense;
		parameters.vuforiaLicenseKey =  vuforiaKey;
		parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

		//  Instantiate the Vuforia engine
		vuforia = ClassFactory.getInstance().createVuforia(parameters);

		// Loading trackables is not necessary for the Tensor Flow Object Detection engine.
	}


	/**
	 * Initialize the Tensor Flow Object Detection engine.
	 */
	private void initTfod(int tfodMonitorViewId)
	{
		TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
		tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
		tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
	}
}
