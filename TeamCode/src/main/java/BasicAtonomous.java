import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ben4toole on 10/20/2016.
 */
@Autonomous(name="EagleBot: Autonomous")
public class BasicAtonomous extends OpMode
{
    EagleBotMethodLibrary AL = new EagleBotMethodLibrary();
    //drivemoter declaration
    DcMotor driveLeft   = null;
    DcMotor  driveRight  = null;
    //servo declaration
    Servo servo = null;
    @Override
    public void init()
    {
        HardwareMap hwMap = super.hardwareMap;
        //initalize drive motors
        driveLeft   = hwMap.dcMotor.get("driveLeft");
        driveRight  = hwMap.dcMotor.get("driveRight");
        // initalize servo
        //servo = hwMap.servo.get("servoExtendRight");
        //correct for motors being on oposite sides
        driveLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        driveRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        // set inital positions and speeds
        driveLeft.setPower(0);
        driveRight.setPower(0);
        //servo.setPosition(0.0);
        // use encoders 1440 ticks pre rev for tetrix 1220 for andymark
        driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start()
    {
        AL.DriveForwardEncoder(70,driveRight,driveLeft);//half the feild forward
        AL.TurnLeftEncoders(90,driveRight,driveLeft); // turn to face becon
        AL.DriveForwardEncoder(68,driveRight,driveLeft); //aproach beacon
    }

    @Override
    public void loop()
    {
        // some code using light sensor to hit all the beacons
    }
}
