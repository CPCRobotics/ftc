import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Eagles FTC on 9/30/2016.
 */
@TeleOp(name="EagleBot: DriverControlled")
public class DriverControlledTankOpmode extends OpMode
{
    //drive motor declaration
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

    }
    @Override
    public void loop()
    {
        double left;
        double right;
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        driveLeft.setPower(left);
        driveRight.setPower(right);

        if(gamepad2.dpad_up)
        {
            driveRight.setPower(1);
            driveLeft.setPower(1);
        }
        if (gamepad2.right_bumper)
        {
            servo.setPosition(1);
        }
    }
}
