package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;

public class Landing {

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
