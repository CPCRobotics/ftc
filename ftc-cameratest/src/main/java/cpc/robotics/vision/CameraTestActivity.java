/*
 * Modified from LASA Robotics sample code.
 * Original source Copyright (c) 2015 LASA Robotics and Contributors
 * MIT licensed
 */

package cpc.robotics.vision;

import android.hardware.Camera;
import android.os.Bundle;
import android.view.WindowManager;

import cpcs.vision.test.R;

import org.lasarobotics.vision.util.ScreenOrientation;

@SuppressWarnings("deprecation")
public class CameraTestActivity extends VisionEnabledActivity {

    private VisionHelper vision;
    private final BlurExtension blur = new BlurExtension();
    private final CropExtension crop = new CropExtension();
    private final JewelsExtension jewels = new JewelsExtension();
    private final ImageRotationExtension rotation = new ImageRotationExtension();
    private final CameraControlExtension cameraControl = new CameraControlExtension();
    private final CameraTestOverlayExtension debugOverlay = new CameraTestOverlayExtension();

    public CameraTestActivity() {
        super();
    }

    /**
     * Called when the activity is first created.
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.activity_cameratest);

        vision = new VisionHelper(this, Camera.CameraInfo.CAMERA_FACING_BACK, 900, 900);
        vision.addExtensions(crop, blur, jewels, rotation, cameraControl, debugOverlay);
        crop.setBounds(-10.0, 0.0, 50.0, 50.0);
        blur.setBlurWidth(5);
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setZeroOrientation(ScreenOrientation.LANDSCAPE_REVERSE);
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE_REVERSE);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
        jewels.enableDebug();

        //
        // Configures VisionHelper to output preview to CameraTestSurfaceView
        //
        initializeVision(R.id.surfaceView, vision);

        //
        // Note that the camera is opened when activity is 'resumed', see base class
        //
    }
}
