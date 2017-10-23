/*
 * Red/Blue Jewel detection code constants.
 */
package cpc.robotics.vision;

import org.lasarobotics.vision.util.color.ColorHSV;

/**
 * Constants about Jewel platform and detection.
 */
public abstract class JewelConstants {
    public static final double JEWEL_DIAMETER = 3.75;   // diameter of jewel in inches
    public static final double JEWEL_DISTANCE = 6.0;    // jewel distance from each other
    public static final double MIN_DISTANCE = 2.0;    // minimum distance between centers
    public static final ColorHSV COLOR_WHITE_LOWER = new ColorHSV(0, 0, (int) (0.500 * 255.0));
    public static final ColorHSV COLOR_WHITE_UPPER = new ColorHSV(255, (int) (0.200 * 255.0), 255);
    public static final ColorHSV COLOR_RED_LOWER = new ColorHSV((int) (350.0 / 360.0 * 255.0), (int) (0.800 * 255.0), (int) (0.200 * 255.0));
    public static final ColorHSV COLOR_RED_UPPER = new ColorHSV((int) (380.0 / 360.0 * 255.0), 255, (int) (0.900 * 255.0));
    public static final ColorHSV COLOR_BLUE_LOWER = new ColorHSV((int) (180.0 / 360.0 * 255.0), (int) (0.300 * 255.0), (int) (0.200 * 255.0));
    public static final ColorHSV COLOR_BLUE_UPPER = new ColorHSV((int) (270.0 / 360.0 * 255.0), 255, (int) (0.900 * 255.0));
}
