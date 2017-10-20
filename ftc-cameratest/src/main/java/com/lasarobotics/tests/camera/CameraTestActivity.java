/*
 * Modified from LASA Robotics sample code.
 * Original source Copyright (c) 2015 LASA Robotics and Contributors
 * MIT licensed
 */

package com.lasarobotics.tests.camera;

import android.os.Bundle;
import android.view.WindowManager;

import org.lasarobotics.vision.opmode.VisionEnabledActivity;

import cpcs.vision.JewelsTestableVisionOpMode;

public class CameraTestActivity extends VisionEnabledActivity {

    private JewelsTestableVisionOpMode opmode;

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

        opmode = new CameraTestVisionOpMode();
        initializeVision(R.id.surfaceView, opmode);
    }

    @Override
    public void onPause() {
        super.onPause();
        opmode.stop();
    }
}