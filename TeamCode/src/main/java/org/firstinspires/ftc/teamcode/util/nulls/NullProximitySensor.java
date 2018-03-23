package org.firstinspires.ftc.teamcode.util.nulls;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.ProximitySensor;

/**
 * Null proximity sensor to replace the AdafruitADPS9960
 */

public class NullProximitySensor implements ProximitySensor {
    @Override
    public double getDistance(DistanceUnit distanceUnit) {
        return 0;
    }

    @Override
    public double getLightDetected() {
        return 0;
    }

    @Override
    public double getRawLightDetected() {
        return 0;
    }

    @Override
    public double getRawLightDetectedMax() {
        return 0;
    }

    @Override
    public void enableLed(boolean b) {

    }

    @Override
    public String status() {
        return null;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
