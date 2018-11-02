package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MineralDetector;
import org.firstinspires.ftc.teamcode.Util.NavUtils;

public class Sampling
{
	private MineralDetector mineralDetector;
	private static final double WALL_TURN_DEGREES = -77;
	private int leftCount = 0;
	private int rightCount = 0;
	private int centerCount = 0;
	private int unknownCount = 0;
	private NavUtils nav;
	private Telemetry telemetry;

	public enum Position {
		NO_DATA,
		UNKNOWN,
		LEFT,
		CENTER,
		RIGHT
	}

	public Sampling(MineralDetector mineralDetector, Telemetry telemetry)
	{
		this.mineralDetector = mineralDetector;
		this.telemetry = telemetry;
	}

	public void locate()
	{
		Position pos = mineralDetector.findMinerals();

		switch (pos)
		{
			case LEFT:
				leftCount++;
				break;

			case CENTER:
				centerCount++;
				break;

			case RIGHT:
				rightCount++;
				break;

			case UNKNOWN:
				unknownCount++;
				break;

			case NO_DATA:
				break;
		}

		//send telemetry about minerals
		if(pos != Position.NO_DATA)
		{
			telemetry.addData("Left Count", leftCount);
			telemetry.addData("Center Count", centerCount);
			telemetry.addData("Right Count", rightCount);
			telemetry.addData("Unknown Count", unknownCount);
			telemetry.update();
		}
	}

	public void Collect( NavUtils nav ) throws InterruptedException
	{
		this.nav = nav;

		if(centerCount > leftCount && centerCount > rightCount)
		{
			CollectCenter();
		}
		else if(leftCount > rightCount)
		{
			CollectLeft();
		}
		else
		{
			CollectRight();
		}
	}

	public void CollectLeft() throws InterruptedException
	{
		nav.samTurn(1, -37.5);
		nav.drive(19, 0.7);
		nav.drive(-19, 0.7);
		//turn towards wall
		nav.samTurn(1, WALL_TURN_DEGREES + 37.5);
	}

	public void CollectCenter() throws InterruptedException
	{
		nav.drive(13, 0.7);
		nav.drive(-13, 0.7);
		//turn towards wall
		nav.samTurn(1, WALL_TURN_DEGREES);
	}

	public void CollectRight() throws InterruptedException
	{
		nav.samTurn(1, 37.5);
		nav.drive(19, 0.7);
		nav.drive(-19, 0.7);
		//turn towards wall
		nav.samTurn(1, -37.5 + WALL_TURN_DEGREES);
	}
}
