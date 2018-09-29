package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.Tilerunner;

/**
 * For controlling the Robot's DcMotors
 */
public class SpeedController {
    private double lastSpeed = 0;
    private final static double SPEED_GAIN = 3;

    public double get(double power) {
        // Make moving the robot more sensitive when the joystick is closer to 0
        double targetPower = power * Math.abs(power);

        // Avoid "deadzone" where the motor doesn't have enough power to move the robot
        targetPower = targetPower * (1 - Tilerunner.MOTOR_DEADZONE) + Tilerunner.MOTOR_DEADZONE * Math.signum(targetPower);

        // Slowly ramp up speed to full speed
        targetPower = (targetPower - lastSpeed) / SPEED_GAIN + lastSpeed;

        return (lastSpeed = targetPower);
    }
}
