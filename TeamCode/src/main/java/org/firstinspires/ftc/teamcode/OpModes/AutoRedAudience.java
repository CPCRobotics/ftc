package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Claiming;
import org.firstinspires.ftc.teamcode.Autonomous.Landing;
import org.firstinspires.ftc.teamcode.Autonomous.Parking;
import org.firstinspires.ftc.teamcode.Autonomous.Sampling;
import org.firstinspires.ftc.teamcode.TileRunner;
import org.firstinspires.ftc.teamcode.Util.IMUSensor;
import org.firstinspires.ftc.teamcode.Util.NavUtils;
import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;


@Autonomous(name="Auto red audience", group="Competition")
public class AutoRedAudience extends LinearOpMode
{
    /* Declare OpMode members. */
    TileRunner         robot   = new TileRunner();

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Auto red audience", "Initializing");
        telemetry.update();
        robot.init( hardwareMap );

        // Initialize the OpModeKeeper Singleton so other parts of our code can use it to get a reference to the OpMode object.
        OpModeKeeper.setOpMode( this );

        // Create an IMUSensor object using the IMU on the robot.
        IMUSensor imu = new IMUSensor( robot.imu );

        // Create the NavUtils object that we will use to drive the robot.
		telemetry.addLine("Initializing NavUtils...");
        NavUtils nav = new NavUtils( robot.leftDrive, robot.rightDrive, imu, 4.0, telemetry );

        // Pause here waiting for the Run button on the driver station to be pressed.
        waitForStart();
        telemetry.addData("CraterSide", "Starting");
        telemetry.update();

        // Start raising the lift (which lowers the robot).
        robot.lift.setPower( 0.8 );

        // Spin here until lift hits upper limit or opmode is stopped.
        while( OpModeKeeper.isActive() && !robot.liftUpperLimit.getState()) { }

        // Stop the motor now that we are on the ground and unlatched from lander.
        robot.lift.setPower( 0 );

        //drive to sampling
        nav.drive(12, 1);

        //sample right
        nav.samTurn(1, 37.5);
        nav.drive(19, 0.7);
        nav.drive(-19, 0.7);

        //reset
        nav.samTurn(1, -37.5);

        //sample middle
        nav.drive(13, 0.7);
        nav.drive(-13, 0.7);

        //sample left
        nav.samTurn(1, -37.5);
        nav.drive(19, 0.7);
        nav.drive(-19, 0.7);

        robot.lift.setPower( -0.8 );
        while( OpModeKeeper.isActive() && !robot.liftLowerLimit.getState()) { }
        robot.lift.setPower( 0 );


        // Spin here updating telemetry until OpMode terminates
        while ( opModeIsActive() )
        {
            telemetry.update();
        }

        // Make sure drive motors are stopped.
        robot.leftDrive.setPower( 0 );
        robot.rightDrive.setPower( 0 );
    }
}
