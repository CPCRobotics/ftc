/*
 * Modified from LASA Robotics sample code.
 * Original source Copyright (c) 2015 LASA Robotics and Contributors
 * MIT licensed
 */

package cpcs.vision;

import android.hardware.Camera;
import android.os.Bundle;
import android.view.WindowManager;

import cpcs.vision.test.R;

import org.lasarobotics.vision.util.ScreenOrientation;

@SuppressWarnings("deprecation")
public class CameraTestActivity extends VisionEnabledActivity {

    private VisionHelper vision;
    private final BlurExtension blur = new BlurExtension();
    private final JewelsExtension jewels = new JewelsExtension();
    private final ImageRotationExtension rotation = new ImageRotationExtension();
    private final CameraControlExtension cameraControl = new CameraControlExtension();
    private final CameraStatsExtension debugOverlay = new CameraStatsExtension();

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

        vision = new VisionHelper(this, Camera.CameraInfo.CAMERA_FACING_BACK, 500, 500);
        vision.addExtensions(blur, jewels, rotation, cameraControl, debugOverlay);
        blur.setBlurWidth(5);
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);
        //rotation.setZeroOrientation(ScreenOrientation.LANDSCAPE_REVERSE);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
        jewels.enableDebug();

        initializeVision(R.id.surfaceView, vision);
    }
}
