import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * Created by Eagles FTC on 9/30/2016.
 */
@TeleOp(name="EagleBot: DriverControlled")
public class DriverControlledOpmode extends OpMode
{
    EagleBotCommon AL = new EagleBotCommon();
    boolean locked =false;

    DcMotor driveLeft   = null;
    DcMotor driveRight  = null;
    DcMotor arm = null;
    DcMotor sweeper = null;
    private ExecutorService executor = null;
    Servo loader  = null;
    Servo ButtionL= null;
    Servo ButtionR = null;
    Servo locker  = null;
    int shootCalls =0;

    Runnable task = null;

    TouchSensor touchSensor;

    @Override
    public void init()
    {
        executor = Executors.newFixedThreadPool(5);

        HardwareMap hwMap = super.hardwareMap;

        touchSensor = hardwareMap.touchSensor.get("touch sensor");

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
        ButtionL = hwMap.servo.get("buttionL");
        ButtionL.setPosition(0);
        ButtionR = hwMap.servo.get("buttionR");
        ButtionR.setPosition(0);
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
        double speedMultiplyer = 0.8;
        int shootCalls = 0;

        driveLeft.setPower( left*speedMultiplyer );
        driveRight.setPower(right*speedMultiplyer);

        sweeper.setPower( gamepad1.right_trigger > 0.1 ? 1 : 0 );
        if (gamepad2.x)
        {
            loader.setPosition(0.9);
        }
        else
        {
            loader.setPosition(0.2);
        }

        if(gamepad2.a &&locked)
        {
            locker.setPosition(1);
            locked=false;
            telemetry.update();
        }
        else if(gamepad2.b && !locked)
        {
            locker.setPosition(0.5);
            locked = true;
            telemetry.update();
        }
        if(gamepad2.right_trigger > 0.1)
        {
            //if this doesnt work make task class level
            //telemetry.addData("Enter shootReload handler","");
            try {
                if(task == null){
                    telemetry.addData("attempt to instantiate task","");
                    task = new ShootAndReload(this);
                }
                telemetry.addData("initiate shooting thread", "iteration: " + shootCalls++);
                executor.execute(task);
            }
            catch (Exception e)
            {
                telemetry.addData("shootReloadersError", e.getMessage());
            }

        }
        if(gamepad2.dpad_left)
        {
            ButtionL.setPosition(1);
        }
        if (gamepad2.dpad_right)
        {
            ButtionR.setPosition(1);
        }
        if(gamepad2.dpad_up)
        {
            executor.shutdown();
            while (!executor.isTerminated())
            {}
        }
        if (gamepad2.dpad_down)
        {
            ButtionR.setPosition(0.1);
            ButtionL.setPosition(0.1);
        }
        if(!touchSensor.isPressed())
        {
            //arm.setPower(gamepad2.left_stick_y);
        }
        else
        {
            /*locker.setPosition(0.5);
            arm.setPower(0);
            locked = true;*/
        }
        telemetry.addData("touch",touchSensor.isPressed());
        telemetry.update();
        //arm.setPower(gamepad2.left_stick_y);

    }

    @Override
    protected void finalize()
    {
        try {
            executor.shutdown();
            while (!executor.isTerminated())
            {}

            super.finalize();
        }
        catch(Throwable ex) {
            //telemetry.addData();
        }

    }
}
