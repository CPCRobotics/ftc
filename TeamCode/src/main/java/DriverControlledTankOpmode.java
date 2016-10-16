import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Eagles FTC on 9/30/2016.
 */
@TeleOp(name="EagleBot: DriverControlled")
public class DriverControlledTankOpmode extends OpMode
{
    DcMotor driveLeft   = null;
    DcMotor  driveRight  = null;
    @Override
    public void init()
    {
        HardwareMap hwMap = super.hardwareMap;

        driveLeft   = hwMap.dcMotor.get("driveLeft");
        driveRight  = hwMap.dcMotor.get("driveRight");

        driveLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        driveRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        driveLeft.setPower(0);
        driveRight.setPower(0);

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
    }
}
