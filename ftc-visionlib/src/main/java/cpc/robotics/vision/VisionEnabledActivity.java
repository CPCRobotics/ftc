package cpc.robotics.vision;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceView;

/**
 * Android Activity class to allow OpenCV UI's to be built around the VisionHelper class.
 */
public abstract class VisionEnabledActivity extends Activity {

    private final static String TAG = "VisionActivity";

    protected VisionHelper vision = null;

    protected VisionEnabledActivity() {

    }

    protected final void initializeVision(int framePreview, VisionHelper vision) {
        this.vision = vision;
        vision.setView((SurfaceView)findViewById(framePreview));
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
    }

    @Override
    public void onPause() {
        super.onPause();
        if (vision != null) {
            Log.d(TAG, "paused");
            vision.disable();
        }
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        if (vision != null) {
            Log.d(TAG, "destroy");
            vision.close();
        }
        this.finish();
    }

    @Override
    protected void onResume() {
        Log.d(TAG, "resume");
        super.onResume();
        if (vision != null) {
            vision.enable();
        }
    }
}
