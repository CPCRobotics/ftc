package cpc.robotics.vision;

import org.lasarobotics.vision.android.Sensors;
import org.lasarobotics.vision.util.FPS;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicLong;

/**
 * Trivial 'extension' to hook into enabled/disable/frame and provide some statistics,
 * including Frames per second, access to sensors, and a frame counter.
 */
public class CameraStatsExtension extends VisionExtension {

    public FPS fps;
    public Sensors sensors;
    protected AtomicLong frame = new AtomicLong();

    public long getFrame() {
        return frame.get();
    }

    @Override
    public void onEnabled() {
        fps = new FPS();
        sensors = new Sensors();
        frame.set(0);
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
        frame.incrementAndGet();
        return rgba;
    }
}
