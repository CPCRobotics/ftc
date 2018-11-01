package org.firstinspires.ftc.teamcode.Util;

public class RobotTilted extends Condition
{
    IMUSensor imu;
    public RobotTilted(IMUSensor imu)
    {
        this.imu = imu;
    }

    public boolean check()
    {
        return imu.getPitch() < -95;
    }
}
