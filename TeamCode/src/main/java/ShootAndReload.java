import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Eagles FTC on 11/26/2016.
 */

public class ShootAndReload implements Runnable
{
    EagleBotHardware hardware;
    public ShootAndReload(EagleBotHardware hw)
    {
        //hardware = new DriverControlledOpmode();
        hardware = hw;
        hardware.telemetry.addData("in Contructors for shootReload", "");
    }
    @Override
    public void run()
    {

        try
        {
            hardware.telemetry.addData("entered run and shoot","Start Run");
            work();
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
    }
    private synchronized void work()
    {
        hardware.telemetry.addData("enter Work method on separate thread", "");
        DcMotor shooter = hardware.arm;
        Servo loader = hardware.loader;
        Servo lock = hardware.locker;
        TouchSensor touchSensor = hardware.armLimitSwitch;
        //unlock to shoot
        lock.setPosition(1);
        try
        {
            Thread.sleep(500);
        }
        catch(Exception e)
        {
            hardware.telemetry.addData("error exception",e.getMessage());
        }
        //shooter.setTargetPosition(2080);//3/4 of a revolution
        //shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setPower(1); // full power reverce
        hardware.telemetry.addData("Wait for touch sensor","");
        while (!touchSensor.isPressed())
        {
        }
        //lock
        hardware.telemetry.addData("TouchSensor is Pressed","");
        loader.setPosition(0.9);
        lock.setPosition(0.5);
        try
        {
            Thread.sleep(500);
        }
        catch(Exception e)
        {
            hardware.telemetry.addData("sleep interupt",e.getMessage());
        }
        shooter.setPower(0);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setTargetPosition(-1440);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setPower(-1);
        hardware.telemetry.addData("Waiting for Shooter", "");
        while (shooter.isBusy()) {
        }
        shooter.setPower(0);
        loader.setPosition(0);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //loader.setPosition(0.5);
        hardware.telemetry.addData("exited run and shoot","released lock");
        try{
            Thread.sleep(1000);
        }
        catch(Exception e)
        {}
        hardware.telemetry.clear();

    }
}
