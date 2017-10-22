package cpcs.vision;

import org.lasarobotics.vision.android.Sensors;
import org.lasarobotics.vision.image.Drawing;
import org.lasarobotics.vision.util.FPS;
import org.lasarobotics.vision.util.color.ColorGRAY;
import org.lasarobotics.vision.util.color.ColorRGBA;
import org.opencv.core.Mat;
import org.opencv.core.Point;

public class CameraTestOverlayExtension extends CameraStatsExtension {

    JewelsExtension jewels = null;
    ImageRotationExtension rotation = null;

    @Override
    public void onEnabled() {
        jewels = vision.getExtension(JewelsExtension.class);
        rotation = vision.getExtension(ImageRotationExtension.class);
    }

    @Override
    public Mat onFrame(Mat rgba) {
        rgba = super.onFrame(rgba);

        //Get jewel analysis
        JewelsDetector.JewelAnalysis analysis = jewels.getAnalysis();

        //Display confidence
        Drawing.drawText(rgba, "Confidence: " + analysis.getConfidenceString(),
                new Point(0, 50), 0.5f, new ColorGRAY(255));

        //Display beacon color
        Drawing.drawText(rgba, analysis.getColorString(),
                new Point(0, 8), 0.5f, new ColorGRAY(255), Drawing.Anchor.BOTTOMLEFT);

        //Display FPS
        Drawing.drawText(rgba, "FPS: " + fps.getFPSString(), new Point(0, 24), 0.5f, new ColorRGBA("#ffffff"));

        //Display JewelsDetector Center
        Drawing.drawText(rgba, "Center: " + jewels.getAnalysis().getCenterString(), new Point(0, 78), 0.5f, new ColorRGBA("#ffffff"));

        //Display inch offset to center
        Drawing.drawText(rgba, "Center Adjust: " + jewels.getAnalysis().getCenterAdjustmentString(),
                new Point(vision.getWidth() - 300, 40), 0.5f, new ColorRGBA("#ffffff"));

        //Display rotation sensor compensation
        Drawing.drawText(rgba, "Rot: " + rotation.getRotationCompensationAngle()
                + " (" + sensors.getScreenOrientation() + ")", new Point(0, 50), 0.5f, new ColorRGBA("#ffffff"), Drawing.Anchor.BOTTOMLEFT); //"#2196F3"

        return rgba;
    }
}
