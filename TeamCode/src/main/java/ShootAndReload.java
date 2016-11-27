import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Eagles FTC on 11/26/2016.
 */

public class ShootAndReload implements Runnable
{

    @Override
    public void run()
    {
        DriverControlledOpmode moters = new DriverControlledOpmode();
        DcMotor shooter = moters.arm;
        Servo loader = moters.loader;
        Servo lock = moters.locker;
        TouchSensor touchSensor = moters.touchSensor;
        //unlock to shoot
        lock.setPosition(1);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //shooter.setTargetPosition(2080);//3/4 of a revolution
        //shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setPower(-1); // full power reverce
        while (!touchSensor.isPressed()) {
        }
        //lock
        lock.setPosition(0.5);
        shooter.setPower(0);
        shooter.setTargetPosition(0);
        //load
        //loader.setPosition(0.8);
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter.setPower(0.7);
        while (shooter.isBusy()) {
        }
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //loader.setPosition(0.5);
    }
}
