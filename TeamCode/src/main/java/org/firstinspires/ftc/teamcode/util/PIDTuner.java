package org.firstinspires.ftc.teamcode.util;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.twigger.Twigger;

import java.util.Locale;

/**
 * Tunes PID live w/ Controls
 */
public class PIDTuner {

    private final double speed;
    private final Gamepad gamepad;
    private final String filename;
    private PIDController.PIDConfiguration pidConfig =
            new PIDController.PIDConfiguration(0.01,0,0);

    private double magnitude = -2; // 10¯²
    private int direction = 1; // Raise, lower

    private final DepressedButton kPButton = new DepressedButton();
    private final DepressedButton kIButton = new DepressedButton();
    private final DepressedButton kDButton = new DepressedButton();
    private final DepressedButton kRaiseButton = new DepressedButton();
    private final DepressedButton kLowerButton = new DepressedButton();
    private final DepressedButton kNegButton = new DepressedButton();
    private final DepressedButton kPosButton = new DepressedButton();
    private final DepressedButton saveButton = new DepressedButton();


    public PIDTuner(String filename, double speed, Gamepad gamepad) {
        this.filename = filename;
        //pidConfig = pidConfig.load(filename);

        this.gamepad = gamepad;
        this.speed = speed;
    }

    public PIDController get() {
        return pidConfig.finish(speed);
    }

    @SuppressLint("DefaultLocale")
    public void update() {
        if (kNegButton.get(gamepad.dpad_left)) direction = -1;
        if (kPosButton.get(gamepad.dpad_right)) direction = 1;
        if (kRaiseButton.get(gamepad.dpad_up)) magnitude++;
        if (kLowerButton.get(gamepad.dpad_down)) magnitude--;

        if (kPButton.get(gamepad.x))
            pidConfig = pidConfig.kP(pidConfig.kP + Math.pow(10, magnitude) * direction);
        if (kIButton.get(gamepad.y))
            pidConfig = pidConfig.kI(pidConfig.kI + Math.pow(10, magnitude) * direction);
        if (kDButton.get(gamepad.b))
            pidConfig = pidConfig.kD(pidConfig.kD + Math.pow(10, magnitude) * direction);

        if (saveButton.get(gamepad.a)) pidConfig.save(filename); // Save to config

        Twigger.getInstance().addLine("Tuner")
                .addData("mag", magnitude)
                .addData("dir", direction)
                .addData("kP", String.format("%6.3g", pidConfig.kP))
                .addData("kI", String.format("%6.3g", pidConfig.kI))
                .addData("kD", String.format("%6.3g", pidConfig.kD));
    }
}
