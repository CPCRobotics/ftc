import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    private EagleBotHardware hardware = new EagleBotHardware();

    /*DcMotor driveLeft   = null;
    DcMotor driveRight  = null;
    DcMotor arm = null;
    DcMotor sweeper = null;*/
    private ExecutorService executor = null;
    /*Servo loader  = null;
    Servo ButtionL= null;
    Servo ButtionR = null;
    Servo locker  = null;*/
    boolean shootonce = true;

    Runnable task = null;

    //TouchSensor touchSensor;

    @Override
    public void init()
    {
        executor = Executors.newFixedThreadPool(5);
        hardware.init(hardwareMap,this.telemetry);
        //HardwareMap hwMap = super.hardwareMap;

        /*touchSensor = hardwareMap.touchSensor.get("touch sensor");

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
        loader.setPosition(0);
        locker = hwMap.servo.get("locker");
        locker.setPosition(1);
        ButtionL = hwMap.servo.get("buttionL");
        ButtionL.setPosition(0);
        ButtionR = hwMap.servo.get("buttionR");
        ButtionR.setPosition(0);*/
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

        hardware.driveLeft.setPower( left*speedMultiplyer );
        hardware.driveRight.setPower(right*speedMultiplyer);

        hardware.sweeper.setPower( gamepad1.right_trigger > 0.1 ? 1 : 0 );
        if (gamepad2.x)
        {
            hardware.loader.setPosition(0.9);
        }
        else
        {
            hardware.loader.setPosition(0);
        }

        if(gamepad2.a &&locked)
        {
            hardware.locker.setPosition(1);
            locked=false;
            telemetry.update();
        }
        else if(gamepad2.b && !locked)
        {
            hardware.locker.setPosition(0.5);
            locked = true;
            telemetry.update();
        }

        if(gamepad2.right_trigger > 0.1 && shootonce)
        {
            shootonce = false;
            telemetry.addData("Enter shootReload handler","");
            try
            {
                if(task == null)
                {
                    telemetry.addData("attempt to instantiate task","");
                    task = new ShootAndReload(hardware);
                }
                telemetry.addData("initiate shooting thread", "iteration: " + shootCalls++);
                executor.execute(task);
            }
            catch (Exception e)
            {
                telemetry.addData("shootReloadersError", e.getMessage());
            }
        }

        if(gamepad2.right_trigger == 0)
        {
            shootonce = true;
        }

        if(gamepad2.dpad_left)
        {
            hardware.ButtionL.setPosition(1);
        }

        if (gamepad2.dpad_right)
        {
            hardware.ButtionR.setPosition(1);
        }

        if (gamepad2.dpad_down)
        {
            hardware.ButtionR.setPosition(0.1);
            hardware.ButtionL.setPosition(0.1);
        }

        telemetry.addData("touch", hardware.armLimitSwitch.isPressed());
        telemetry.update();


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
