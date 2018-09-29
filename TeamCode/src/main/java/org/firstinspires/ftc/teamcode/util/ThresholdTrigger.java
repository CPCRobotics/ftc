package org.firstinspires.ftc.teamcode.util;

/**
 * Trigger button with a threshold.
 *
 * If the threshold isn't reached, it goes down to 0.
 */
public class ThresholdTrigger {
    private static final double DEFAULT_THRESHOLD = 0.2;
    private final double threshold;

    public ThresholdTrigger(double threshold) {
        this.threshold = threshold;
    }

    public ThresholdTrigger() {
        this(DEFAULT_THRESHOLD);
    }

    public double get(double val) {
        if (Math.abs(val) > threshold)
            return val;

        return 0;
    }
}
