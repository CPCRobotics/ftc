package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.strategy.CryptoboxColumn;
import org.firstinspires.ftc.teamcode.strategy.JewelDirection;

/**
 * Judge mode for evaluating the features of the robot
 */
@TeleOp(name="Judge Mode", group="Judge")
public class JudgeMode extends OpMode {

    private Tilerunner tilerunner = new Tilerunner();
    private double whackerPosition = 0;

    @Override
    public void init() {
        tilerunner.init(hardwareMap, telemetry, true);
    }

    @Override
    public void loop() {
        // Glyph Grabber
        if (gamepad1.left_trigger >= TankDrive.JOYSTICK_THRESHOLD) {
            tilerunner.ejectGlyph(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger >= TankDrive.JOYSTICK_THRESHOLD) {
            tilerunner.grabGlyph(gamepad1.right_trigger);
        } else {
            tilerunner.grabGlyph(0);
        }

        // Lift
        if (Math.abs(gamepad2.left_stick_y) >= TankDrive.JOYSTICK_THRESHOLD) {
            tilerunner.setLiftPower(TankDrive.calculateLiftSpeed(gamepad2.left_trigger));
        }

        // Jewel Whacker
        if (Math.abs(gamepad2.right_stick_y) >= TankDrive.JOYSTICK_THRESHOLD) {
            whackerPosition += TankDrive.WHACKER_CONTROL_SPEED * gamepad2.right_stick_y;
            whackerPosition = TankDrive.withinRange(0, 1, whackerPosition);
            tilerunner.jewelWhacker.setPosition(whackerPosition);
        }

        // Jewel Status
        if (gamepad1.a) {
            // Fake autonomous status 1
            CryptoboxColumn.LEFT.displayPosition(tilerunner);
            JewelDirection.LEFT.displayStatus(tilerunner);
        } else if (gamepad1.b) {
            // Fake autonomous status 2
            CryptoboxColumn.CENTER.displayPosition(tilerunner);
            JewelDirection.RIGHT.displayStatus(tilerunner);
        } else if (gamepad1.x) {
            // Fake autonomous status 3
            CryptoboxColumn.RIGHT.displayPosition(tilerunner);
            JewelDirection.LEFT.displayStatus(tilerunner);
        } else if (gamepad1.y) {
            // Fake autonomous status 4 (unknown)
            tilerunner.displayUnknown();
        } else {
            tilerunner.displayStatus();
        }
    }
}
