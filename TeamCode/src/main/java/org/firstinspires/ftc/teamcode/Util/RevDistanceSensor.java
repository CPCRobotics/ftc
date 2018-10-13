package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RevDistanceSensor
{
    //work in progress
    private DistanceSensor sensorRange;

    public RevDistanceSensor(HardwareMap hardwareMap)
    {
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
    }
}
