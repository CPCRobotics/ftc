/*
 * Red/Blue Jewel detection code, wrapped as an 'extension'.
 *
 * Based on LASA Robotics Beacon detection code.
 * Original source Copyright (c) 2016 Arthur Pachachura, LASA Robotics, and contributors
 * MIT licensed
 */
package cpcs.vision;

import org.lasarobotics.vision.opmode.VisionOpModeCore;
import org.lasarobotics.vision.opmode.extensions.VisionExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Mat;

/**
 * Extension that supports finding and reading jewel color data
 */
public class JewelsExtension implements VisionExtension {
    private JewelsDetector jewels;

    private JewelsDetector.JewelAnalysis analysis = new JewelsDetector.JewelAnalysis();

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
    public void init(VisionOpModeCore opmode) {
        //Initialize all detectors here
        jewels = new JewelsDetector();
    }

    @Override
    public void loop(VisionOpModeCore opmode) {

    }

    @Override
    public Mat frame(VisionOpModeCore opmode, Mat rgba, Mat gray) {
        try {
            //Get screen orientation data
            ScreenOrientation orientation = ScreenOrientation.getFromAngle(
                    VisionOpModeCore.rotation.getRotationCompensationAngle());

            //Get color analysis
            this.analysis = jewels.analyzeFrame(rgba, gray, orientation);

        } catch (Exception e) {
            e.printStackTrace();
        }

        return rgba;
    }

    @Override
    public void stop(VisionOpModeCore opmode) {

    }
}