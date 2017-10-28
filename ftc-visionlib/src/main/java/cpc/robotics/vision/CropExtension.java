/*
 * Extension to blur the image. This is l
 */
package cpc.robotics.vision;

import org.lasarobotics.vision.detection.objects.Rectangle;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * Extension that takes original image and crops it to reduce camera area
 */
public class CropExtension extends VisionExtension {

    // cropping region as a percentage.
    private double centerX = 0; // 0 is natural center, -ve = left, +ve = right -50 to +50
    private double centerY = 0; // 0 is natural center, -ve = left, +ve = right -50 to +50
    private double width = 100; // 100 is entire width
    private double height = 100; // 100 is entire height
    private SizeI viewSize;
    private Rect roiRect;

    public void setBounds(double cX, double cY, double w, double h) {
        this.centerX = cX;
        this.centerY = cY;
        this.width = w;
        this.height = h;
    }

    @Override
    public SizeI resize(SizeI prevSize) {
        viewSize = new SizeI((int)Math.max(1.0, prevSize.width * this.width / 100.0),
                (int)Math.max(1.0, prevSize.height * this.height / 100.0));
        Point roiOffset = new Point((this.centerX + 50.0) * prevSize.width / 100.0 - viewSize.width/2,
                (this.centerY+50.0) * prevSize.height / 100.0 - viewSize.height/2);
        Size roiSize = new Size(viewSize.width, viewSize.height);
        roiRect = new Rect(roiOffset, roiSize);
        return viewSize;
    }

    /**
     * Apply the opencv medianBlur algorithm.
     * @param img input matrix
     * @return modified matrix containing blurred image
     */
    @Override
    public Mat onFrame(Mat img) {
        if (roiRect == null) {
            return img;
        }
        try {
            Mat roiImg = new Mat(img, roiRect);
            roiImg.copyTo(img);
            return img;
        } catch (Exception e) {
            e.printStackTrace();
            return img;
        }
    }
}
