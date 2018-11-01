package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.TileRunner;
import org.firstinspires.ftc.teamcode.Util.IMUSensor;
import org.firstinspires.ftc.teamcode.Util.NavUtils;
import org.firstinspires.ftc.teamcode.Util.RobotTilted;

public class Parking {

	public static void ParkInCrater( NavUtils nav )
	{
		RobotTilted robotTilted = new RobotTilted(nav.imu);
		nav.driveCondition(robotTilted, -1);
	}
}
