package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Landing;
import org.firstinspires.ftc.teamcode.Autonomous.Sampling;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.TileRunner;
import org.firstinspires.ftc.teamcode.Util.IMUSensor;
import org.firstinspires.ftc.teamcode.Util.NavUtils;
import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;


@Autonomous(name="Prematch", group="Test")
public class prematch extends LinearOpMode
{
	/* Declare OpMode members. */
	TileRunner         robot   = new TileRunner();

	@Override
	public void runOpMode() throws InterruptedException
	{
		robot.init( hardwareMap );
		telemetry.addData("Prematch", "Initialized");
		telemetry.update();

		// Pause here waiting for the Run button on the driver station to be pressed.
		waitForStart();

		robot.lift.setPower(-1);
		while(!robot.liftLowerLimit.getState() && opModeIsActive()) {}
		robot.lift.setPower(0);
		telemetry.addLine("Lift lowered");
		telemetry.update();

//		robot.arm.setPower(-0.1);
//		sleep(500);
//		robot.lift.setPower(0);
//		telemetry.addLine("Arm reset");
//		telemetry.update();

		telemetry.addLine("Robot ready");
		telemetry.addLine("Go win driveteam!");
		telemetry.update();
	}
}
