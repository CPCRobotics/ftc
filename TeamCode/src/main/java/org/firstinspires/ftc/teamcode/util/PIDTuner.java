package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Tunes PID live w/ Controls
 */
public class PIDTuner {
    private final double speed;
    private final Gamepad gamepad;
    private final String filename;

    private double kP = 0.01;
    private double kI = 0.01;
    private double kD = 0.00;

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
        this.gamepad = gamepad;
        this.speed = speed;
    }

    public PIDController get() {
        return new PIDController(speed, kP, kI, kD);
    }

    public void update() {
        if (kNegButton.get(gamepad.dpad_left)) direction = -1;
        if (kPosButton.get(gamepad.dpad_right)) direction = 1;
        if (kRaiseButton.get(gamepad.dpad_up)) magnitude++;
        if (kLowerButton.get(gamepad.dpad_down)) magnitude--;

        if (kPButton.get(gamepad.x)) kP += Math.pow(10, magnitude) * direction;
        if (kIButton.get(gamepad.y)) kI += Math.pow(10, magnitude) * direction;
        if (kDButton.get(gamepad.b)) kD += Math.pow(10, magnitude) * direction;
        if (saveButton.get(gamepad.a)) get().save(filename); // Save to config

        Twigger.getInstance().addLine("PID")
                .addData("mag", magnitude)
                .addData("dir", direction)
                .addData("kP", kP)
                .addData("kI", kI)
                .addData("kD", kD);
    }
}
