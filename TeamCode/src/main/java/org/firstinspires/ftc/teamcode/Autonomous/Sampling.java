package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Util.NavUtils;

public class Sampling
{
	private static final double WALL_TURN_DEGREES = -77;
	static NavUtils nav;
	public enum Position {
		LEFT,
		CENTER,
		RIGHT
	}

	public static void Collect( Position mineralPosition, NavUtils nav ) throws InterruptedException
	{
		Sampling.nav = nav;

		//drive to sampling position
		nav.drive(12, 1);

		switch ( mineralPosition )
		{
			case LEFT:
				CollectLeft() ;
				break;

			case CENTER:
				CollectCenter();
				break;

			case RIGHT:
				CollectRight();
				break;
		}
	}

	public static void CollectLeft() throws InterruptedException
	{
		nav.samTurn(1, -37.5);
		nav.drive(19, 0.7);
		nav.drive(-19, 0.7);
		//turn towards wall
		nav.samTurn(1, WALL_TURN_DEGREES + 37.5);
	}

	public static void CollectCenter() throws InterruptedException
	{
		nav.drive(13, 0.7);
		nav.drive(-13, 0.7);
		//turn towards wall
		nav.samTurn(1, WALL_TURN_DEGREES);
	}

	public static void CollectRight() throws InterruptedException
	{
		nav.samTurn(1, 37.5);
		nav.drive(19, 0.7);
		nav.drive(-19, 0.7);
		//turn towards wall
		nav.samTurn(1, -37.5 + WALL_TURN_DEGREES);
	}
}
