/*
 * Extension to blur the image
 */
package cpcs.vision;

import org.lasarobotics.vision.detection.objects.Rectangle;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * Extension that supports finding and reading jewel color data
 */
public class BlurExtension extends VisionExtension {
    private int blurWidth = 5;

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
