package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Autonomous.Sampling;
import org.firstinspires.ftc.teamcode.R;

import java.util.List;

public class MineralDetector
{
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    Telemetry telemetry;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public MineralDetector(Telemetry telemetry, String vuforiaKey, int tfodMonitorViewId)
    {
        this.telemetry = telemetry;
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia(vuforiaKey);

        if (ClassFactory.getInstance().canCreateTFObjectDetector())
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

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */

    public Sampling.Position findMinerals()
    {
        Sampling.Position result = Sampling.Position.CENTER;
        if (tfod != null)
        {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null)
            {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 3)
                {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions)
                    {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                        {
                            goldMineralX = (int) recognition.getLeft();
                        }
                        else if (silverMineral1X == -1)
                        {
                            silverMineral1X = (int) recognition.getLeft();
                        }
                        else
                        {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1)
                    {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X)
                        {
                            telemetry.addData("Gold Mineral Position", "Left");
                            result = Sampling.Position.LEFT;
                        }
                        else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X)
                        {
                            telemetry.addData("Gold Mineral Position", "Right");
                            result = Sampling.Position.RIGHT;
                        }
                        else
                        {
                            telemetry.addData("Gold Mineral Position", "Center");
                            result = Sampling.Position.CENTER;
                        }
                    }
                }
            }
        }
        return result;
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
        parameters.cameraDirection = CameraDirection.BACK;

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
