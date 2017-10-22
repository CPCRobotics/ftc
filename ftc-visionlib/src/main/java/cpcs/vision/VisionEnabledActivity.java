package cpcs.vision;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceView;

import org.lasarobotics.vision.android.Sensors;
import org.lasarobotics.vision.util.FPS;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.core.Mat;

/**
 * Initiates a VisionEnabledActivity
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
            vision.disableView();
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
            vision.enableView();
        }
    }
}
