import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ben4toole on 10/20/2016.
 */

public class AutonomousMethodLibrary
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
}
