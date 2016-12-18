import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Created by ben4toole on 10/20/2016.
 */
@Autonomous(name="EagleBot: Autonomous Left position")
public class AutonomousOpModeLeftPos extends OpMode
{
    boolean looponce = true;
    private ExecutorService executor = null;
    Runnable task = null;
    boolean cs = true;
    EagleBotCommon AL = new EagleBotCommon();
    //drivemoter declaration
    DcMotor driveLeft   = null;
    DcMotor  driveRight  = null;
    //servo declaration
    Servo locker = null;
    Servo ButtionL = null;
    Servo ButtionR = null;

    ColorSensor colorSensor = null;
    ColorSensor BW = null;

    HardwareMap hwMap = null;
    @Override
    public void init()
    {
        executor = Executors.newFixedThreadPool(2);
        hwMap = super.hardwareMap;
        //initalize drive motors
        driveLeft   = hwMap.dcMotor.get("driveLeft");
        driveRight  = hwMap.dcMotor.get("driveRight");
        //correct for motors being on oposite sides
        driveLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        driveRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
         //set inital positions and speeds
        driveLeft.setPower(0);
        driveRight.setPower(0);
        // use encoders 1440 ticks pre rev for tetrix 1220 for andymark
        driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        locker = hwMap.servo.get("locker");
        ButtionL =hwMap.servo.get("buttionL");
        ButtionR=hwMap.servo.get("buttionR");

        colorSensor=hwMap.colorSensor.get("ColorSensor");
        BW=hwMap.colorSensor.get("bw");
        //locker.setPosition(0.5);
        ButtionL.setPosition(0.1);
        ButtionR.setPosition(0);
    }

    @Override
    public void start()
    {

       //AL.DriveForwardEncoder(6,driveRight,driveLeft);//half the feild forward
       // AL.TurnLeftEncoders(91,driveRight,driveLeft); // turn to face becon
        //AL.DriveForwardEncoder(68,driveRight,driveLeft); //aproach beacon*/
    }

    @Override
    public void loop()
    {
        if(looponce)
        {
            telemetry.addData("attempt to instantiate task", "");
            //task = new ShootAndReload(new DriverControlledOpmode());
            //executor.execute(new ShootAndReload(this);
           // executor.execute(new ShootAndReload(new DriverControlledOpmode()));
          /*  try {
                Thread.sleep(5000);
            } catch (Exception e) {
                telemetry.addData("error exception", e.getMessage());
            }*/
            AL.DriveForwardEncoder(43, driveRight, driveLeft);
            telemetry.addData("drove forward"," trying to turn");
            AL.TurnLeftEncoders(50, driveRight, driveLeft);
            telemetry.addData("turned"," left");
            AL.TurnRightEncoders(50, driveRight, driveLeft);
            telemetry.addData("turnd right"," trying to move");
            AL.DriveForwardEncoder(8, driveRight, driveLeft);
            telemetry.addData("moved","");
            looponce = false;
        }
            /*colorSensor.enableLed(true);
        BW.enableLed(true);
        if(cs)
        {
            telemetry.addData("BW blue", BW.blue());
            telemetry.addData("BW red", BW.red());
            telemetry.addData("BW green", BW.green());
            telemetry.addData("cs blue", colorSensor.blue());
            telemetry.addData("cs red", colorSensor.red());
            telemetry.addData("cs green", colorSensor.green());
            cs = false;
        }
        telemetry.update();
        if (BW.red() > 0 && BW.blue() > 0 && BW.green() > 0 && BW.red() <  0 && BW.blue() > 0 && BW.green() > 0 )
        {
            if (colorSensor.red() > colorSensor.blue())
            {
                ButtionR.setPosition(1);
            }
            else
            {
                ButtionL.setPosition(1);
            }
        }*/
        //Servo locker = hwMap.servo.get("locker");
       // locker.setPosition(0.5);
        // some code using light sensor to hit all the beacons
    }
}
