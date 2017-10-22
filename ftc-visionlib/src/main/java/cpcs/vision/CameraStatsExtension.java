package cpcs.vision;

import org.lasarobotics.vision.android.Sensors;
import org.lasarobotics.vision.util.FPS;
import org.opencv.core.Mat;

public class CameraStatsExtension extends VisionExtension {

    public FPS fps;
    public Sensors sensors;

    @Override
    public void onEnabled() {
        fps = new FPS();
        sensors = new Sensors();
    }

    @Override
    public void onDisabled() {
        if (sensors != null) {
            sensors.stop();
            sensors = null;
            fps = null;
        }
    }

    @Override
    public Mat onFrame(Mat rgba) {
        fps.update();
        return rgba;
    }
}
