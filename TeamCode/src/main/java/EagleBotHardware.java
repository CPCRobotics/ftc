import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Eagles FTC on 12/17/2016.
 */

public class EagleBotHardware
{
    public DcMotor driveLeft   = null;
    public DcMotor driveRight  = null;
    public DcMotor arm = null;
    public DcMotor sweeper = null;
    public Servo loader  = null;
    public Servo ButtionL= null;
    public Servo ButtionR = null;
    public Servo locker  = null;
    public TouchSensor armLimitSwitch;
    public Telemetry telemetry = null;

    HardwareMap hwMap =null;

    public void EagleBotHardware()
    {
    }

    public void init(HardwareMap ahwMap, Telemetry atelemetry)
    {
        telemetry = atelemetry;
        hwMap = ahwMap;

        armLimitSwitch = hwMap.touchSensor.get("touch sensor");

        driveLeft   = hwMap.dcMotor.get("driveLeft");
        driveRight  = hwMap.dcMotor.get("driveRight");

        driveLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        driveRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        driveLeft.setPower(0);
        driveRight.setPower(0);

        driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Setup sweeper motor
        sweeper = hwMap.dcMotor.get("sweeper");
        sweeper.setDirection( DcMotor.Direction.REVERSE );
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        sweeper.setPower(0);

        arm = hwMap.dcMotor.get("arm");
        arm.setDirection( DcMotor.Direction.REVERSE );
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        arm.setPower(0);

        loader = hwMap.servo.get("loader");
        loader.setPosition(0);
        locker = hwMap.servo.get("locker");
        locker.setPosition(1);
        ButtionL = hwMap.servo.get("buttionL");
        ButtionL.setPosition(0);
        ButtionR = hwMap.servo.get("buttionR");
        ButtionR.setPosition(0);
    }
}
