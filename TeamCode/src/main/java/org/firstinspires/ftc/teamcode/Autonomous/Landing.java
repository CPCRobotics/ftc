package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;

/**
 * Code to handle the Landing operation of autonomous for Rover Ruckus.
 */
public class Landing {

	/**
	 * Lower the robot to the mat and unlatch from the lander.
	 *
	 * @param motor - DcMotor object for the robot lift mechanism.
	 * @param upperLimit - DigitalChannel object used to read the state of the lifts upper limit switch.
	 */
    public static void Land(DcMotor motor, DigitalChannel upperLimit)
    {
    	// Start raising the lift (which lowers the robot).
        motor.setPower( 0.8 );

        // Spin here until lift hits upper limit or opmode is stopped.
        while( OpModeKeeper.isActive() && !upperLimit.getState())
        {}

        // Stop the motor now that we are on the ground and unlatched from lander.
        motor.setPower( 0 );
    }
}
