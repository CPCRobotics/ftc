package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.Landing;
import org.firstinspires.ftc.teamcode.TileRunner;
import org.firstinspires.ftc.teamcode.Util.IMUSensor;
import org.firstinspires.ftc.teamcode.Util.NavUtils;
import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;


@Autonomous(name="Crater Side", group="Test")
public class CraterSide extends LinearOpMode {


    /* Declare OpMode members. */
    TileRunner         robot   = new TileRunner();

    @Override
    public void runOpMode()
            throws InterruptedException
    {
        telemetry.addData("Initializing", "Initializing");
        telemetry.update();
        robot.init( hardwareMap );

        // Initialize the OpModeKeeper Singleton so other parts of our code can use it to get a reference to the OpMode object.
        OpModeKeeper.setOpMode( this );

        // Create an IMUSensor object using the IMU on the robot.
        IMUSensor imu = new IMUSensor( robot.imu );


        telemetry.addLine("About to init nav util");
        // Create the NavUtils object that we will use to drive the robot.
        NavUtils nav = new NavUtils( robot.leftDrive, robot.rightDrive, imu, 4.0, telemetry );

        // Pause here waiting for the Run button on the driver station to be pressed.
        waitForStart();
        telemetry.addData("Running", "Running");
        telemetry.update();

        Landing.Land( robot.lift, robot.liftUpperLimit);

/*
        nav.drive( 10, .5);

        nav.samTurn(.5, -70);

        nav.drive(40,.5);
        nav.samTurn(.5, -60);
        nav.drive(60,.5);
        
        nav.drive(-80,.5);

        //nav.drive( 1000 );*/


        while ( opModeIsActive() ) {
            telemetry.update();
        }

        // Stop all motors
        robot.leftDrive.setPower( 0 );
        robot.rightDrive.setPower( 0 );
    }
}
