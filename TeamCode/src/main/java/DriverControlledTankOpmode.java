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
    EagleBotMethodLibrary AL = new EagleBotMethodLibrary();
    boolean locked =false;
    DcMotor driveLeft   = null;
    DcMotor driveRight  = null;
    DcMotor arm = null;
    DcMotor sweeper = null;

    Servo loader  = null;
    Servo buttion  = null;
    Servo locker  = null;
    double servovalue = 0.0;

    @Override
    public void init()
    {
        HardwareMap hwMap = super.hardwareMap;

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
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        arm.setPower(0);

        loader = hwMap.servo.get("loader");
        loader.setPosition(0.1);
        locker = hwMap.servo.get("locker");
        locker.setPosition(1);
        buttion = hwMap.servo.get("buttion");
        buttion.setPosition(0.9);
    }

    @Override
    public void start()
    {

    }

    @Override
    public void loop()
    {
        telemetry.addData("Locked:",locked);
        //loader.setPosition(gamepad1.right_stick_y);
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;

        driveLeft.setPower( left*0.8 );
        driveRight.setPower(right*0.8);

        sweeper.setPower( gamepad1.right_trigger > 0.1 ? 1 : 0 );
        if (gamepad2.x)
        {
            loader.setPosition(0.9);
        }
        else
        {
            loader.setPosition(0.1);
        }

        if(gamepad2.a && !locked)
        {
            locker.setPosition(1);
            locked=true;
            telemetry.update();
        }
        else if(locked&&gamepad2.b)
        {
            locker.setPosition(0.5);
            locked = false;
            telemetry.update();
        }
        if(gamepad2.dpad_down)
        {
            //AL.ShootAndReload(arm, loader, locker);
            buttion.setPosition(1);
        }
        if(gamepad2.dpad_up)
        {
            buttion.setPosition(0);
        }
        arm.setPower(gamepad2.left_stick_y);

    }
}
