package org.firstinspires.ftc.teamcode.Util;

/**
 * Feigns continuous angles, i.e. angle measurements beyond the [-180°, 180°] range
 */
public final class VirtualCompass {
    private static final double ANGLE_CROSS_THRESHOLD = 180;

    private final IMUSensor imu;

    private double lastRawAngle = 0;
    private int offset = 0; // Offset number of revolutions
    private boolean firstTimeRun = true;

    public VirtualCompass(IMUSensor imu) {
        this.imu = imu;
    }

    public void updateCompass() {
        double currAngle = imu.getHeading();

        if (!firstTimeRun) {
            double diff = lastRawAngle - currAngle;
            int rotation = 0;
            if (diff < -ANGLE_CROSS_THRESHOLD) rotation = -1;
            if (diff > ANGLE_CROSS_THRESHOLD) rotation = 1;

            offset += rotation;
        }

        firstTimeRun = false;
        lastRawAngle = currAngle;
    }

    public double getAngle() {
        updateCompass();
        return imu.getHeading() + offset*360;
    }
}
