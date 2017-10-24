/*
 * Show result of selecting a specific color range
 */
package cpc.robotics.colorcheck;

import org.lasarobotics.vision.detection.ColorBlobDetector;
import org.lasarobotics.vision.detection.objects.Contour;
import org.lasarobotics.vision.detection.objects.Rectangle;
import org.lasarobotics.vision.image.Drawing;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.lasarobotics.vision.util.color.ColorHSV;
import org.lasarobotics.vision.util.color.ColorRGBA;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import java.util.List;

import cpc.robotics.vision.ImageRotationExtension;
import cpc.robotics.vision.JewelConstants;
import cpc.robotics.vision.JewelsDetector;
import cpc.robotics.vision.VisionExtension;

/**
 * Extension that supports finding and reading jewel color data
 */
public class FeedbackExtension extends VisionExtension {

    private ImageRotationExtension rotation = null;
    private ColorHSV lower = new ColorHSV(0,0,0);
    private ColorHSV upper = new ColorHSV(255,255,255);

    public FeedbackExtension() {
    }

    public void setRange(ColorHSV lower, ColorHSV upper) {
        this.lower = lower;
        this.upper = upper;
    }

    @Override
    public String toString() {
        return String.format("HSV Range %s to %s", lower.toString(), upper.toString());
    }

    @Override
    public void onEnabled() {
        // Vision has been enabled
        rotation = vision.getExtension(ImageRotationExtension.class);
        if (rotation == null) {
            rotation = new ImageRotationExtension(); // if not using rotation
        }
    }

    @Override
    public Mat onFrame(Mat img) {
        try {
            final ColorBlobDetector colorDetector = new ColorBlobDetector(lower, upper);
            final ScreenOrientation orientation = ScreenOrientation.getFromAngle(
                    rotation.getRotationCompensationAngle());
            colorDetector.process(img);
            List<Contour> contours = colorDetector.getContours();
            Drawing.drawContours(img, contours, new ColorRGBA("#FF00FF"), 2);

        } catch (Exception e) {
            e.printStackTrace();
        }

        return img;
    }
}