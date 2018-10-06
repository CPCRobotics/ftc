package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public final class IMUSensor {

    private final BNO055IMU imu;

    private double heading = 0;
    private double roll = 0;
    private double pitch = 0;

    private double addDegree = 0;


    public IMUSensor( BNO055IMU imu )
	{
        this.imu = imu;

    }

    /**
	 * Read the angle date (heading, roll and pitch) from the IMU and cache it in private members.
	 *
     * @return void
     */
    private void readAngles()
	{
        Orientation angles =  imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        heading = angles.firstAngle;
        roll = angles.secondAngle;
        pitch = angles.thirdAngle;
    }

    public double getHeading()
	{
        readAngles();
        return heading;
    }

	public double getRoll()
	{
		readAngles();
		return roll;
	}

	public double getPitch()
	{
		readAngles();
		return pitch;
	}
}
