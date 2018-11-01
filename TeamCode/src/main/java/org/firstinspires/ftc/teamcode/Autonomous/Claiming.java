package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Util.NavUtils;
import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;


public class Claiming
{
	public static void DeployMarker( NavUtils nav, DcMotor armMotor )
	{
		armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		armMotor.setTargetPosition(1500);
		armMotor.setPower(0.75);
		while(armMotor.isBusy() && OpModeKeeper.isActive()) {}
	}
}
