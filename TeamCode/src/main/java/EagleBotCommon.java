import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by ben4toole on 10/20/2016.
 */

public class EagleBotCommon
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

    public void ShootAndReload()//DcMotor shooter, Servo loader, Servo lock, TouchSensor touchSensor
    {
        //Thread Shoot = new Thread(new ShootAndReload());
        //Shoot.start();
/*
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
        };*/
      }

    //drive by encoders
    // use encoders 1440 ticks pre rev for tetrix 1220 or 1120 for andymark
    //ticks needed = 1440*dist/wheal circumference
    public void DriveForwardEncoder(int dist, DcMotor DR, DcMotor DL)
    {
        //constants to calculate dist in ticks assuming dist and cir are same units
        int cir = 12,tickConstant=1440,ticks;

        //calculate dist in ticks
        ticks = tickConstant*dist/cir;

        // set encode values to 0
        DR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set both motors to run for distance calculated in ticks
        DR.setTargetPosition(ticks);
        DL.setTargetPosition(ticks);
        DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //run full speed forward
        DR.setPower(1);
        DL.setPower(1);
        while (DR.isBusy())
    {
    }
    }
    public void DriveBackwardEncoders(int dist, DcMotor DR, DcMotor DL)
    {
        //constants to calculate dist in ticks assuming dist and cir are same units
        int cir = 12,tickConstant=1440,ticks;

        //calculate dist in ticks
        ticks = tickConstant*dist/cir;

        // set encode values to 0
        DR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set both motors to run for distance calculated in ticks
        DR.setTargetPosition(ticks);
        DL.setTargetPosition(ticks);
        DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //run full speed in reverse
        DR.setPower(-1);
        DL.setPower(-1);
        while (DR.isBusy())
        {
        }
    }
    //******if 1/2 rev in each wheal results in a 90 deg turn then A(deg)=A/180 rev************
    //ticks needed = A/180*1440
    public void TurnRightEncoders(int angle, DcMotor DR, DcMotor DL)
    {
        //constants to calculate dist in ticks assuming dist and cir are same units
        //calculate dist in ticks
        int ticks = (angle/18)*1440;

        // set encode values to 0
        DR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set both motors to run for distance calculated in ticks
        DR.setTargetPosition(ticks);
        DL.setTargetPosition(ticks);
        DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //run full speed in reverse
        DR.setPower(-1);
        DL.setPower(1);
        while (DR.isBusy())
        {
        }

    }
    public void TurnLeftEncoders(int angle, DcMotor DR, DcMotor DL)
    {
        //constants to calculate dist in ticks assuming dist and cir are same units
        int ticks=(angle/18)*1440;;
        // set encode values to 0
        DR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set both motors to run for distance calculated in ticks
        DR.setTargetPosition(ticks);
        DL.setTargetPosition(ticks);
        DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //run full speed in reverse
        DR.setPower(1);
        DL.setPower(-1);
        while (DR.isBusy())
        {
        }

    }
}
