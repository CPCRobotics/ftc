package org.firstinspires.ftc.teamcode;

import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.lasarobotics.vision.util.ScreenOrientation;

import cpc.robotics.vision.BlurExtension;
import cpc.robotics.vision.CameraControlExtension;
import cpc.robotics.vision.CameraStatsExtension;
import cpc.robotics.vision.ImageRotationExtension;
import cpc.robotics.vision.JewelsDetector;
import cpc.robotics.vision.JewelsExtension;
import cpc.robotics.vision.VisionExtension;
import cpc.robotics.vision.VisionHelper;

@Autonomous(name="TestOpenCV" )
@Disabled
public class TestOpenCV extends LinearOpMode {
    //Frame counter
    protected final BlurExtension blur = new BlurExtension();
    protected final JewelsExtension jewels = new JewelsExtension();
    protected final ImageRotationExtension rotation = new ImageRotationExtension();
    protected final CameraControlExtension cameraControl = new CameraControlExtension();
    protected final CameraStatsExtension cameraStats = new CameraStatsExtension();

    protected VisionHelper openCvBlock(VisionExtension detection) {
        VisionHelper vision = new VisionHelper(hardwareMap.appContext, Camera.CameraInfo.CAMERA_FACING_BACK, 500, 500);
        vision.addExtensions(blur);
        vision.addExtensions(detection);
        vision.addExtensions(rotation, cameraControl);
        vision.addExtensions(cameraStats);

        blur.setBlurWidth(5);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);
        //rotation.setZeroOrientation(ScreenOrientation.LANDSCAPE_REVERSE);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
        vision.enable();
        return vision;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //Wait for the match to begin
        waitForStart();

        // when done like this, openCv camera will be released at the end of the try block
        // as used here, this try block isn't very useful other than to ensure camera is correctly
        // shut down at end of run.
        try(VisionHelper vision = openCvBlock(jewels)) {

            //Main loop
            //Camera frames and OpenCV analysis will be delivered to this method as quickly as possible
            //This loop will exit once the opmode is closed
            while (opModeIsActive()) {
                //Log a few things
                JewelsDetector.JewelAnalysis analysis = jewels.getAnalysis();
                telemetry.addData("Jewel Colors", analysis.getColorString());
                telemetry.addData("Jewel Center", analysis.getCenterString());
                telemetry.addData("Jewel Confidence", analysis.getConfidenceString());
                telemetry.addData("Center Adjustment", analysis.getCenterAdjust());
                telemetry.addData("Screen Rotation", rotation.getScreenOrientationActual());
                telemetry.addData("Frame Rate", cameraStats.fps.getFPSString() + " FPS");
                telemetry.addData("Frame Size", "Width: " + vision.getWidth() + " Height: " + vision.getHeight());
                telemetry.addData("Frame Counter", cameraStats.getFrame());
                telemetry.update();
                sleep(1); // Allow other processes to run
            }
        }
    }
}
