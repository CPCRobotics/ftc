package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;


public class NavUtils {
	DcMotor		leftMotor = null;
	DcMotor 	rightMotor = null;
	BNO055IMU	imu = null;


	public NavUtils( DcMotor left, DcMotor right, BNO055IMU imu ) {
		leftMotor = left;
		rightMotor = right;
		this.imu = imu;
	}


	/**
	 * Drive the robot straight forwards or backwards a given distance.
	 *
	 * @param distance - Distance to drive in inches, negative values drive backwards.
	 */
	public void drive( int distance )
	{

	}

	/**
	 * Turn the robot in place a specified number of degrees.
	 *
	 * @param angle - Number of degrees to turn the robot, positive values turn right, negative values turn left.
	 */
	public void turn( int angle )
	{

	}
}
