/*
 * Red/Blue Jewel detection code,.
 *
 * Based on LASA Robotics Beacon detection code.
 * Original source Copyright (c) 2016 Arthur Pachachura, LASA Robotics, and contributors
 * MIT licensed
 */
package cpcs.vision;

import org.lasarobotics.vision.detection.ColorBlobDetector;
import org.lasarobotics.vision.detection.objects.Contour;
import org.lasarobotics.vision.detection.objects.Rectangle;
import org.lasarobotics.vision.image.Drawing;
import org.lasarobotics.vision.util.MathUtil;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.lasarobotics.vision.util.color.ColorRGBA;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.text.DecimalFormat;
import java.util.List;

/**
 * JewelsDetector location and analysis
 */
public final class JewelsDetector {

    private ColorBlobDetector blueDetector = new ColorBlobDetector(JewelConstants.COLOR_BLUE_LOWER, JewelConstants.COLOR_BLUE_UPPER);
    private ColorBlobDetector redDetector = new ColorBlobDetector(JewelConstants.COLOR_RED_LOWER, JewelConstants.COLOR_RED_UPPER);
    private ColorBlobDetector whiteDetector = new ColorBlobDetector(JewelConstants.COLOR_WHITE_LOWER, JewelConstants.COLOR_WHITE_UPPER);
    private boolean debug = false;

    /**
     * Instantiate a jewel detector
     */
    public JewelsDetector() {
    }

    /**
     * Analyze the current frame using the selected analysis method
     *
     * @param img  Image to analyze
     * @param gray Grayscale image to analyze
     * @return JewelsDetector analysis class
     */
    public JewelAnalysis analyzeFrame(Mat img, Mat gray) {
        return analyzeFrame(img, gray, ScreenOrientation.LANDSCAPE);
    }

    /**
     * Analyze the current frame using the selected analysis method
     *
     * @param img         Image to analyze
     * @param gray        Grayscale image to analyze
     * @param orientation Screen orientation compensation, given by the android.Sensors class
     * @return JewelsDetector analysis class
     */
    public JewelAnalysis analyzeFrame(Mat img, Mat gray, ScreenOrientation orientation) {
        //Figure out which way to read the image
        double orientationAngle = orientation.getAngle();
        boolean swapLeftRight = orientationAngle >= 180; //swap if LANDSCAPE_WEST or PORTRAIT_REVERSE
        boolean readOppositeAxis = orientation == ScreenOrientation.PORTRAIT ||
                orientation == ScreenOrientation.PORTRAIT_REVERSE; //read other axis if any kind of portrait
        Rectangle bounds = new Rectangle(img.size());
        //Bound the image
        if (readOppositeAxis)
            //Force the analysis box to transpose inself in place
            //noinspection SuspiciousNameCombination
            bounds = new Rectangle(
                    new Point(bounds.center().y / img.height() * img.width(),
                            bounds.center().x / img.width() * img.height()),
                    bounds.height(), bounds.width()).clip(new Rectangle(img.size()));
        if (!swapLeftRight && readOppositeAxis)
            //Force the analysis box to flip across its primary axis
            bounds = new Rectangle(
                    new Point((img.size().width / 2) + Math.abs(bounds.center().x - (img.size().width / 2)),
                            bounds.center().y), bounds.width(), bounds.height());
        else if (swapLeftRight && !readOppositeAxis)
            //Force the analysis box to flip across its primary axis
            bounds = new Rectangle(
                    new Point(bounds.center().x, img.size().height - bounds.center().y),
                    bounds.width(), bounds.height());
        bounds = bounds.clip(new Rectangle(img.size()));

        //Get contours within the bounds
        redDetector.process(img);
        blueDetector.process(img);
        whiteDetector.process(img);
        List<Contour> contoursRed = redDetector.getContours();
        List<Contour> contoursBlue = blueDetector.getContours();
        List<Contour> contoursWhite = whiteDetector.getContours();

        //DEBUG Draw contours before filtering
        if (debug) {
            Drawing.drawContours(img, contoursRed, new ColorRGBA("#FF0000"), 2);
            Drawing.drawContours(img, contoursBlue, new ColorRGBA("#0000FF"), 2);
            Drawing.drawContours(img, contoursWhite, new ColorRGBA("#FFFF00"), 2);
            Drawing.drawRectangle(img, bounds, new ColorRGBA("#aaaaaa"), 4);
        }

        //Get the largest contour in each - we're calling these the jewels
        Contour largestRed = findLargest(contoursRed);
        Contour largestBlue = findLargest(contoursBlue);
        if (largestBlue == null || largestRed == null) {
            return new JewelsDetector.JewelAnalysis();
        }

        if (debug) {
            Drawing.drawRectangle(img, bounds, new ColorRGBA("#aa00aa"), 4);
        }

        // We'll be more selective with white - we can see reflections in the balls appear as white
        Contour redBounding = outerContour(largestRed);
        Contour blueBounding = outerContour(largestBlue);

        Contour largestWhite = findLargest(contoursWhite, redBounding, blueBounding);
        Contour whiteBounding = outerContour(largestWhite);
        if (debug) {
            Drawing.drawContour(img, redBounding, new ColorRGBA("#800000"));
            Drawing.drawContour(img, blueBounding, new ColorRGBA("#000080"));
            if (whiteBounding != null) {
                Drawing.drawContour(img, whiteBounding, new ColorRGBA("#808000"));
            }
        }

        Point bestRedCenter = redBounding.center();
        Point bestBlueCenter = blueBounding.center();
        Point bestWhiteCenter = null;
        if (whiteBounding != null) {
            bestWhiteCenter = whiteBounding.center();
        }

        //DEBUG R/B text
        if (debug) {
            Drawing.drawText(img, "R", bestRedCenter, 1.0f, new ColorRGBA(255, 0, 0));
            Drawing.drawText(img, "B", bestBlueCenter, 1.0f, new ColorRGBA(0, 0, 255));
            if (bestWhiteCenter != null) {
                Drawing.drawText(img, "W", bestWhiteCenter, 1.0f, new ColorRGBA(255, 255, 0));
            }
            Drawing.drawRectangle(img, bounds, new ColorRGBA("#00aaaa"), 4);
        }

        // pixels per inch
        double redFactor = ((redBounding.height() + redBounding.width()) / 2) / JewelConstants.JEWEL_DIAMETER;
        double blueFactor = ((blueBounding.height() + blueBounding.width()) / 2) / JewelConstants.JEWEL_DIAMETER;
        double scaleFactor = (redFactor + blueFactor) / 2;
        double minWidth = scaleFactor * JewelConstants.MIN_DISTANCE;

        double whiteToBlue, whiteToRed, center, confidence = 1.0;
        if (bestWhiteCenter == null) {
            whiteToBlue = (bestBlueCenter.x - bestRedCenter.x) / 2;
            whiteToRed = -whiteToBlue;
            center = bestBlueCenter.x-whiteToBlue;
            confidence = 0.00;
        } else {
            center = bestWhiteCenter.x;
            whiteToBlue = bestBlueCenter.x - center;
            whiteToRed = bestRedCenter.x - center;
        }
        if (Math.abs(whiteToBlue) < minWidth || Math.abs(whiteToRed) < minWidth) {
            confidence = Math.min(confidence, 0.1);
        } else {
            double ratio = Math.abs(whiteToBlue / whiteToRed);
            if (ratio < 1.0) {
                ratio = 1.0 / ratio;
            }
            double distance = Math.abs(whiteToBlue) + Math.abs(whiteToRed) / ratio;
            double desiredDistance = scaleFactor * JewelConstants.JEWEL_DISTANCE;
            confidence = Math.min(confidence, distance / desiredDistance);
        }
        // This may help allow robot to reposition itself
        double centerAdjust = (center - bounds.width()/2 - bounds.left()) / scaleFactor;
        if (swapLeftRight) {
            centerAdjust = -centerAdjust;
            whiteToBlue = -whiteToBlue;
            whiteToRed = -whiteToRed;
        }

        if (whiteToBlue < 0.0 && whiteToRed > 0.0) {
            return new JewelsDetector.JewelAnalysis(JewelColor.BLUE, JewelColor.RED, bestWhiteCenter, centerAdjust, confidence);
        } else if (whiteToBlue > 0.0 && whiteToRed < 0.0) {
            return new JewelsDetector.JewelAnalysis(JewelColor.RED, JewelColor.BLUE, bestWhiteCenter, centerAdjust, confidence);
        } else {
            return new JewelsDetector.JewelAnalysis();
        }
    }

    private static Contour findLargest(List<Contour> contours, Contour ... exclusions) {
        Contour largest = null;
        double maxArea = 0.0f;
        for (Contour c : contours) {
            if (c.area() > maxArea && !isInsideExclusion(c, exclusions)) {
                largest = c;
                maxArea = c.area();
            }
        }
        return largest;
    }

    private static boolean isInsideExclusion(Contour test, Contour ... exclusions) {
        //
        // Take center of the test contour. See if the point (center) is in any
        // of the exclusions.
        //

        if (exclusions.length == 0) {
            // Return early to avoid unnecessary test.centroid()
            return false;
        }

        Point center = test.centroid();
        for (Contour exclusion : exclusions) {
            if (Imgproc.pointPolygonTest(exclusion.getDoubleData(), center, false) > 0) {
                return true;
            }
        }
        return false;
    }

    /**
     * Connects all the outermost points ignoring inner points, to determine outermost contour.
     * Also known as a hull.
     * @param contour Original contour.
     * @return New contour (hull)
     */
    private static Contour outerContour(Contour contour) {
        if (contour == null) {
            return contour;
        }
        MatOfInt hullIndexMat = new MatOfInt();
        MatOfPoint contourMat = contour.getData();
        int height = contourMat.height();
        if (height == 0) {
            return contour;
        }
        Imgproc.convexHull(contourMat, hullIndexMat);
        Point[] newPoints = new Point[hullIndexMat.height()];
        int[] data = new int[1];
        for (int i = 0; i < newPoints.length; i++) {
            hullIndexMat.get(i, 0, data);
            newPoints[i] = new Point(contourMat.get(data[0], 0));
        }
        return new Contour(new MatOfPoint(newPoints));
    }

    /**
     * Enable debug displays.
     * Use this only on testing apps, otherwise it might slow your program
     * down a little bit or confuse your custom code
     */
    public void enableDebug() {
        this.debug = true;
    }

    /**
     * Disable debug displays (default)
     */
    public void disableDebug() {
        this.debug = false;
    }

    /**
     * JewelsDetector color struct
     */
    public enum JewelColor {
        RED,
        BLUE,
        UNKNOWN;

        @Override
        public String toString() {
            switch (this) {
                case RED:
                    return "red";
                case BLUE:
                    return "blue";
                case UNKNOWN:
                default:
                    return "???";
            }
        }
    }

    /**
     * JewelsDetector analysis struct
     */
    public static class JewelAnalysis {
        private final double confidence;
        private final JewelColor left;
        private final JewelColor right;
        private final Point center;
        private final double centerAdjust;

        /**
         * Instantiate a blank analysis
         */
        public JewelAnalysis() {
            this.left = JewelColor.UNKNOWN;
            this.right = JewelColor.UNKNOWN;
            this.confidence = 0.0f;
            this.center = null;
            this.centerAdjust = 0.0;
        }

        JewelAnalysis(JewelColor left, JewelColor right, Point center, double centerAdjust, double confidence) {
            this.left = left;
            this.right = right;
            this.confidence = confidence;
            this.centerAdjust = centerAdjust;
            this.center = center;
        }

        /**
         * Get the center line of the jewels
         *
         * @return Rectangle
         */
        public Point getCenter() {
            return center;
        }

        /**
         * Get an adjustment needed in inches to get to the
         * center - only valid for high confidence values.
         *
         * @return adjustment in inches
         */
        public double getCenterAdjust() { return centerAdjust; }

        /**
         * Get the color of the left jewel
         *
         * @return JewelsDetector color
         */
        public JewelColor getLeftColor() {
            return left;
        }

        /**
         * Get the color of the right jewel
         *
         * @return JewelsDetector color
         */
        public JewelColor getRightColor() {
            return right;
        }

        /**
         * Get a confidence value that the jewel analysis is correct
         * <p/>
         * This is an approximation, but can be used carefully to filter out random noise.
         * <p/>
         * Also, only certain analysis methods provide a confidence - this will then return zero.
         *
         * @return Confidence, if applicable - zero if not applicable
         */
        public double getConfidence() {
            return MathUtil.coerce(0.0, 1.0, confidence);
        }

        /**
         * Get a confidence string that the jewel analysis is correct
         * <p/>
         * This is an approximation, but can be used carefully to filter out random noise.
         * <p/>
         * Also, only certain analysis methods provide a confidence - this will then return zero.
         *
         * @return Confidence
         */
        public String getConfidenceString() {
            final DecimalFormat format = new DecimalFormat("0.000");
            return format.format(MathUtil.coerce(0, 1, getConfidence()) * 100.0f) + "%";
        }

        /**
         * <p>
         * This is an approximate number of inches to adjust to align center of the jewels
         * </p>
         * @return center adjustment in inches as a string.
         */
        public String getCenterAdjustmentString() {
            final DecimalFormat format = new DecimalFormat("0.00");
            return format.format(centerAdjust) + "in";
        }

        /**
         * Get a string representing the colors of the jewels
         *
         * @return left, right
         */
        public String getColorString() {
            return left.toString() + ", " + right.toString();
        }

        /**
         * Get the location of the jewels as a string
         *
         * @return Center of the jewels
         */
        public String getCenterString() {
            return String.valueOf(getCenter());
        }

        @Override
        public String toString() {
            return "Color: " + getColorString() + "\r\n Center: " + getCenterString() + " (" + getCenterAdjustmentString() + ") " + "\r\n Confidence: " + getConfidenceString();
        }
    }
}
