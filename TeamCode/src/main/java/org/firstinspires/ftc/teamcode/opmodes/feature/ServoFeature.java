package org.firstinspires.ftc.teamcode.opmodes.feature;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Generic servo feature
 */
public class ServoFeature extends Feature {
    private final Servo servo;
    private final double speed;

    private double position = 0;

    public ServoFeature(Servo servo, double speed) {
        this.servo = servo;
        this.speed = speed;

        reset();
    }

    private static double constrain(double val) {
        if (val < 0) return 0;
        if (val > 1) return 1;
        return val;
    }

    private void reset() {
        position = servo.getPosition();
    }

    @Override
    public boolean call(double value) {
        position = constrain(position + speed * value);
        servo.setPosition(value);

        return (value != 0);
    }
}
