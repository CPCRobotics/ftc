/**
 *  Singleton class to make a reference to the current running OpMode available
 *  to all modules without having to pass it around as a parameter everywhere.
 */

package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class OpModeKeeper
{
	private static OpMode instance = null;

	// Private constructor to keep anyone from instantiating the Singleton.
	private OpModeKeeper() {}


	// Call this method to retrieve the reference to the running OpMode
	public static OpMode getOpMode()
	{
		return instance;
	}


	// This method should be called once to initialize the OpMode reference.
	public static void setOpMode( OpMode opmode ){
		instance = opmode;
	}


	/**
	 * Check to see if the OpMode we reference is still active (and should continue running).  Since
	 * only LinearOpModes have the opModeIsActive method we have to first check to see if the OpMode
	 * we have is an instance of that class, if not we always return true (on the assumption that
	 * this is TeleOp OpMode).
	 *
	 * @return - Boolean indicating whether OpMode is still active.
	 */
	public static boolean isActive()
	{
		if ( instance instanceof LinearOpMode )
		{
			return ((LinearOpMode)(instance)).opModeIsActive();
		}
		else
		{
			return true;
		}
	}
}
