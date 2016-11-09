import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ben4toole on 10/20/2016.
 */

public class EagleBotMethodLibrary
{
    public void DriveForward(int time,int speed, DcMotor DR, DcMotor DL)
    {
        DR.setPower(speed);
        DL.setPower(-speed);
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void DriveBackward(int time,int speed, DcMotor DR, DcMotor DL)
    {
        DR.setPower(-speed);
        DL.setPower(speed);
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void TurnRight(int time,int speed, DcMotor DR, DcMotor DL) throws InterruptedException
    {
        try {
            DR.setPower(-speed);
            DL.setPower(-speed);
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void TurnLeft(int time,int speed, DcMotor DR, DcMotor DL) throws InterruptedException
    {
        DR.setPower(speed);
        DL.setPower(speed);
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void ShootAndReload(DcMotor shooter, Servo loader, Servo lock)
    {
        //unlock to shoot
        lock.setPosition(0.1);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setTargetPosition(-8);//unknown amount of encoder ticks to lock
        shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //lock
        lock.setPosition(0.8);
        //load
        loader.setPosition(0.8);
        loader.setPosition(0.1);
    }
}
