package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Util.OpModeKeeper;

public class Landing {

    public static void Land(DcMotor motor, DigitalChannel upperLimit)
    {
        motor.setPower(.3);
        while(OpModeKeeper.isActive()){
            if (upperLimit.getState() == true){
                motor.setPower(0);
                break;
            }
        }
    }
}
