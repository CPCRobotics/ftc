/*
 * Extension to blur the image. This is l
 */
package cpc.robotics.vision;

import org.lasarobotics.vision.detection.objects.Rectangle;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * Extension that supports finding and reading jewel color data
 */
public class BlurExtension extends VisionExtension {
    public static final int BLUR_NONE = 1; // Blur 1-pixel x 1-pixel
    public static final int BLUR_LITTLE = 3; // Blur 3-pixels x 3-pixels
    public static final int BLUR_MORE = 5; // Blur 5-pixels x 5-pixels
    public static final int BLUR_LOTS = 7; // Blur 7-pixels x 7-pixels - very slow

    private int blurWidth = BLUR_MORE;

    /**
     * Get current blur width
     *
     * @return Width of blur kernel
     */
    public int getBlurWidth() {
        return blurWidth;
    }

    /**
     * Change blur width
     *
     * @param blurWidth width of blur, expected to be an odd number
     */
    public void setBlurWidth(int blurWidth) {
        this.blurWidth = blurWidth;
    }

    /**
     * Apply the opencv medianBlur algorithm.
     * @param img input matrix
     * @return modified matrix containing blurred image
     */
    @Override
    public Mat onFrame(Mat img) {
        Mat output = img;
        try {
            Rectangle bounds = new Rectangle(img.size());
            Mat blurred = img.clone();
            Imgproc.medianBlur(img, blurred, blurWidth);
            output = blurred;
        } catch (Exception e) {
            e.printStackTrace();
            output = img;
        }
        return output;
    }
}
