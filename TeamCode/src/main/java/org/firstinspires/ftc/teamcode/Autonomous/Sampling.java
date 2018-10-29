package org.firstinspires.ftc.teamcode.Autonomous;

public class Sampling
{
	public enum Position {
		LEFT,
		CENTER,
		RIGHT
	}

	public static void Collect( Position mineralPosition )
	{
		switch ( mineralPosition )
		{
			case LEFT:
				CollectLeft();
				break;

			case CENTER:
				CollectCenter();
				break;

			case RIGHT:
				CollectRight();
				break;
		}
	}

	public static void CollectLeft()
	{

	}

	public static void CollectCenter()
	{

	}

	public static void CollectRight()
	{

	}
}
