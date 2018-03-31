package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Feigns continuous angles, i.e. angle measurements beyond the [-180°, 180°] range
 */
public class VirtualCompass {
    private static final double ANGLE_CROSS_THRESHOLD = 180;

    private final BNO055IMU imu;

    private double lastRawAngle = 0;
    private double offset = 0; // Offset number of revolutions
    private boolean firstTimeRun = true;

    public VirtualCompass(BNO055IMU imu) {
        this.imu = imu;
    }

    /**
     * @return angle between -180° and 180°
     */
    private double getRawAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void updateCompass() {
        if (!firstTimeRun) {
            double diff = lastRawAngle - getRawAngle();
            if (diff < -ANGLE_CROSS_THRESHOLD)
                offset -= 1;
            else if (diff > ANGLE_CROSS_THRESHOLD)
                offset += 1;
        }

        firstTimeRun = false;
        lastRawAngle = getRawAngle();
    }

    public double getAngle() {
        updateCompass();
        return getRawAngle() + offset*360;
    }
}
