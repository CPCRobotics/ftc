package org.firstinspires.ftc.teamcode.Autonomous;

import java.io.PipedOutputStream;
import java.util.List;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Util.NavUtils;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;



public class Sampling
{
	private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
	private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
	private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
	private static final double WALL_TURN_DEGREES = -77;

	private static final int POSITION_ONE = 0;
	private static final int POSITION_TWO = 360;
	private static final int POSITION_THREE = 720;
	private static final int MIDPOINT_ONE = (POSITION_TWO - POSITION_ONE) / 2;
	private static final int MIDPOINT_TWO = (POSITION_THREE - POSITION_TWO) / 2 + POSITION_TWO;

	private NavUtils nav = null;
	private Telemetry telemetry = null;
	private VuforiaLocalizer vuforia = null;
	private TFObjectDetector tfod = null;
	private ElapsedTime timer = null;

	// Buffer to store our position history in.
	private static final int POSITION_BUFFER_SIZE = 10;
	private int posBufferIndex = 0;
	private Position posBuffer[];

	public enum Position {
		NULL,
		UNKNOWN,
		LEFT,
		CENTER,
		RIGHT
	};

	public Sampling( NavUtils nav, Telemetry telemetry, String vuforiaKey, int tfodMonitorViewId, WebcamName camera )
	{
		this.nav = nav;
		this.telemetry = telemetry;


		// Allocate the position buffer and initialize it.
		posBuffer = new Position[ POSITION_BUFFER_SIZE ];
		for ( int n = 0; n < POSITION_BUFFER_SIZE; n++ )
		{
			posBuffer[n] = Position.NULL;
		}

		timer = new ElapsedTime( ElapsedTime.Resolution.MILLISECONDS );

		// Create the VuforiaLocalizer that will be used to capture frames of video.
		initVuforia(vuforiaKey, camera );

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
		if ( timer.milliseconds() < 250 )
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

			//if (objectCount == 3 )
			//{
				int gold1X = -1;
				int silver1X = -1;
				int silver2X = -1;

				for (Recognition recognition : recognitions)
				{
					if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
					{
						gold1X = (int) recognition.getLeft();
					}
					else if ( recognition.getLabel().equals(LABEL_SILVER_MINERAL) && silver1X == -1 )
					{
						silver1X = (int) recognition.getLeft();
					}
					else if ( recognition.getLabel().equals(LABEL_SILVER_MINERAL) && silver2X == -1 )
					{
						silver2X = (int) recognition.getLeft();
					}
					else
					{
						telemetry.addLine( "****** Unknown object ****" );
					}
				}

				// NOTE: The left and right are swapped because of our camera position
				if (gold1X != -1 && silver1X != -1 && silver2X != -1)
				{
					telemetry.addData("silver1 x", silver1X);
					telemetry.addData("silver2 x" ,silver2X);

					//int silverPosition = silver1X + silver2X;

					if(silver1X < MIDPOINT_TWO && silver2X < MIDPOINT_TWO)
					{
						result = Sampling.Position.RIGHT;
						telemetry.addData( "Current Position", "RIGHT");
					}
					else if((silver1X > MIDPOINT_TWO && silver2X < MIDPOINT_ONE) || (silver2X > MIDPOINT_TWO && silver1X < MIDPOINT_ONE))
					{
						result = Sampling.Position.CENTER;
						telemetry.addData( "Current Position", "CENTER");
					}
					else
					{
						result = Sampling.Position.LEFT;
						telemetry.addData( "Current Position", "LEFT");
					}

//					if (gold1X < silver1X && gold1X < silver2X)
//					{
//						result = Sampling.Position.LEFT;
//						telemetry.addData( "Current Position", "LEFT");
//					}
//					else if (gold1X > silver1X && gold1X > silver2X)
//					{
//						result = Sampling.Position.RIGHT;
//						telemetry.addData( "Current Position", "RIGHT");
//					}
//					else if((gold1X > silver1X && gold1X < silver2X) || (gold1X < silver1X && gold1X > silver2X))
//					{
//						result = Sampling.Position.CENTER;
//						telemetry.addData( "Current Position", "CENTER");
//					}
//					else
//					{
//						result = Sampling.Position.UNKNOWN;
//						telemetry.addData( "Current Position", "UNKNOWN");
//					}

					// Store this position in the position buffer and advance the buffer index to the next entry.
					posBuffer[ posBufferIndex ] = result;
					posBufferIndex = ++posBufferIndex % POSITION_BUFFER_SIZE;

					checkBuffer();
				}
				else
				{
					telemetry.addLine("Unknown");
				}
				telemetry.update();
			//}
		}
	}

	public Position checkBuffer()
	{
		Position result = Position.NULL;
		int leftCount = 0;
		int rightCount = 0;
		int centerCount = 0;
		int unknownCount = 0;

		for ( int n = 0; n < POSITION_BUFFER_SIZE; n++ )
		{
			switch ( posBuffer[n] )
			{
				case LEFT:
					leftCount++;
					break;

				case RIGHT:
					rightCount++;
					break;

				case CENTER:
					centerCount++;
					break;

				case UNKNOWN:
					unknownCount++;
					break;
			}
		}

		if ( (leftCount > rightCount) && (leftCount > centerCount) && ( leftCount > unknownCount))
		{
			result = Position.LEFT;
		}
		else if ( (rightCount > centerCount) && ( rightCount > unknownCount))
		{
			result = Position.RIGHT;
		}
		else if ( centerCount > unknownCount)
		{
			result = Position.CENTER;
		}
		else
		{
			result = Position.UNKNOWN;
		}

		telemetry.addData( "leftCount", leftCount );
		telemetry.addData( "centerCount", centerCount );
		telemetry.addData( "rightCount", rightCount );
		telemetry.addData( "unknownCount", unknownCount );

		return result;
	}


	public void CraterCollect() throws InterruptedException
	{
		switch ( checkBuffer() )
		{
			case LEFT:
				CollectLeft();
				break;

			case RIGHT:
				CollectRight();
				break;

			case CENTER:
				CollectCenter();
				break;

			case UNKNOWN:
				CollectCenter();
				break;
		}

	}
	public void DepoCollect() throws InterruptedException
	{
		switch ( checkBuffer() )
		{
			case LEFT:
				DepoCollectLeft();
				break;

			case RIGHT:
				CollectRight();
				break;

			case CENTER:
				DepoCollectCenter();
				break;

			case UNKNOWN:
				CollectCenter();
				break;
		}

	}


	private void CollectLeft () throws InterruptedException
	{
		nav.samTurn(1, -37.5);
		nav.drive(19, 0.7);
		nav.drive(-19, 0.7);
		//turn towards wall
		nav.samTurn(1, WALL_TURN_DEGREES + 37.5);
	}


	private void CollectCenter () throws InterruptedException
	{
		nav.drive(13, 0.7);
		nav.drive(-13, 0.7);
		//turn towards wall
		nav.samTurn(1, WALL_TURN_DEGREES);
	}


	private void CollectRight () throws InterruptedException
	{
		nav.samTurn(1, 37.5);
		nav.drive(19, 0.7);
		nav.drive(-19, 0.7);
		//turn towards wall
		nav.samTurn(1, -37.5 + WALL_TURN_DEGREES);
	}

	private void DepoCollectLeft () throws InterruptedException
	{
		nav.samTurn(1, -37.5);
		nav.drive(19 + 6, 0.7);
		nav.drive(-19 - 6, 0.7);
		//turn towards wall
		nav.samTurn(1, WALL_TURN_DEGREES + 37.5);
	}


	private void DepoCollectCenter () throws InterruptedException
	{
		nav.drive(13 + 23, 0.7);
		nav.drive(-13 - 23, 0.7);
		//turn towards wall
		nav.samTurn(1, WALL_TURN_DEGREES);
	}



	/**
	 * Initialize the Vuforia localization engine.
	 */
	private void initVuforia(String vuforiaKey, WebcamName camera )
	{
		/*
		 * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
		 */
		VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

		//parameters.vuforiaLicenseKey = vuphoriaLicense;
		parameters.vuforiaLicenseKey =  vuforiaKey;

		// If caller provided an external camera name then use it otherwise use phone's built in camera.
		if ( camera != null )
		{
			parameters.cameraName = camera;
		}
		else
		{
			parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
		}

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
