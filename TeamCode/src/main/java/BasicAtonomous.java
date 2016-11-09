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
        servo = hwMap.servo.get("servoExtendRight")
        //correct for motors being on oposite sides
        driveLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        driveRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        // set inital positions and speeds
        driveLeft.setPower(0);
        driveRight.setPower(0);
        servo.setPosition(0.0);
        // use encoders
        driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start()
    {
        AL.DriveForward(5000,1,driveRight,driveLeft);
        try
        {
            AL.TurnRight(2000,1,driveRight,driveLeft);
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }
        AL.DriveForward(5000,1,driveRight,driveLeft);
    }

    @Override
    public void loop()
    {
    }
}
