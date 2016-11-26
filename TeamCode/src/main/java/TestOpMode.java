import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import android.hardware.SensorManager;

/**
 * Created by ben4toole on 10/20/2016.
 */
@Autonomous(name="EagleBot: Test")
public class TestOpMode extends OpMode
{
    EagleBotCommon AL = new EagleBotCommon();
    //drivemoter declaration
    DcMotor driveLeft   = null;
    DcMotor  driveRight  = null;
    //servo declaration
    Servo locker = null;
    HardwareMap hwMap = null;
    @Override
    public void init()
    {
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
        //locker = hwMap.servo.get("locker");
       // Servo locker = hwMap.servo.get("locker");
        //locker.setPosition(0.5);
    }

    @Override
    public void start()
    {
        //HardwareMap hwMap = super.hardwareMap;
        //Servo locker = hwMap.servo.get("locker");
        //locker.setPosition(0.5);
        telemetry.addData("Starting",driveLeft);
       AL.DriveForwardEncoder(6,driveRight,driveLeft);//half the feild forward
        AL.TurnLeftEncoders(91,driveRight,driveLeft); // turn to face becon
        //AL.DriveForwardEncoder(68,driveRight,driveLeft); //aproach beacon*/


    }

    @Override
    public void loop()
    {
        //Servo locker = hwMap.servo.get("locker");
       // locker.setPosition(0.5);
        // some code using light sensor to hit all the beacons
    }
}
