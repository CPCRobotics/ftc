/*
 * Copyright (c) 2016 Arthur Pachachura, LASA Robotics, and contributors
 * MIT licensed
 */
package org.lasarobotics.vision.opmode.extensions;

import org.lasarobotics.vision.opmode.VisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpModeCore;
import org.opencv.core.Mat;

/**
 * Interface for vision extensions for JewelsVisionOpMode
 */
public interface VisionExtension {
    void init(VisionOpModeCore opmode);

    void loop(VisionOpModeCore opmode);

    Mat frame(VisionOpModeCore opmode, Mat rgba, Mat gray);

    void stop(VisionOpModeCore opmode);
}
