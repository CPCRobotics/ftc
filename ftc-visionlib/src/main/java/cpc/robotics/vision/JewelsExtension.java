/*
 * Red/Blue Jewel detection code, wrapped as an 'extension'.
 *
 * Based on LASA Robotics Beacon detection code.
 * Original source Copyright (c) 2016 Arthur Pachachura, LASA Robotics, and contributors
 * MIT licensed
 */
package cpc.robotics.vision;

import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;

/**
 * Extension that supports finding and reading jewel color data
 */
public class JewelsExtension extends VisionExtension {
    private JewelsDetector jewels;

    private JewelsDetector.JewelAnalysis analysis = new JewelsDetector.JewelAnalysis();
    private ImageRotationExtension rotation = null;

    private JewelsDetector.JewelAnalysis bestAnalysis = analysis;

    public JewelsExtension() {
    }

    /**
     * Get latest jewel analysis
     *
     * @return A JewelsDetector.JewelAnalysis struct
     */
    public JewelsDetector.JewelAnalysis getAnalysis() {
        return analysis;
    }

    /**
     * Enable debug drawing. Use this on testing apps only, not the robot controller.
     */
    public void enableDebug() {
        jewels.enableDebug();
    }

    /**
     * Disable debug drawing (default). Use this on the robot controller.
     */
    public void disableDebug() {
        jewels.disableDebug();
    }

    @Override
    public void onCreate() {
        // Create detector object(s).
        jewels = new JewelsDetector();
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
    public Mat onFrame(Mat rgba) {
        try {
            //Get screen orientation data
            ScreenOrientation orientation = ScreenOrientation.getFromAngle(
                    rotation.getRotationCompensationAngle());

            //Get color analysis
            this.analysis = jewels.analyzeFrame(rgba, orientation);

            if (analysis.getConfidence() > bestAnalysis.getConfidence())
                bestAnalysis = analysis;

        } catch (Exception e) {
            e.printStackTrace();
        }

        return rgba;
    }

    public JewelsDetector.JewelAnalysis getBestAnalysis() {
        return bestAnalysis;
    }
}