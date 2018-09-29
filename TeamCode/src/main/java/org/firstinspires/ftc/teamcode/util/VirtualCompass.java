package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Feigns continuous angles, i.e. angle measurements beyond the [-180°, 180°] range
 */
public final class VirtualCompass {
    private static final double ANGLE_CROSS_THRESHOLD = 180;

    private final BNO055IMU imu;

    private double lastRawAngle = 0;
    private int offset = 0; // Offset number of revolutions
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
        double currAngle = getRawAngle();

        if (!firstTimeRun) {
            double diff = lastRawAngle - currAngle;
            int rotation = 0;
            if (diff < -ANGLE_CROSS_THRESHOLD) rotation = -1;
            if (diff > ANGLE_CROSS_THRESHOLD) rotation = 1;

            offset += rotation;


            Twigger.getInstance().addLine("compass")
                    .addData("last", lastRawAngle)
                    .addData("curr", currAngle)
                    .addData("diff", diff)
                    .addData("status", rotation)
                    .done();
        }

        firstTimeRun = false;
        lastRawAngle = currAngle;
        Twigger.getInstance().addLine("compass")
                .addData("curr", currAngle);
    }

    public double getAngle() {
        updateCompass();
        return getRawAngle() + offset*360;
    }
}
