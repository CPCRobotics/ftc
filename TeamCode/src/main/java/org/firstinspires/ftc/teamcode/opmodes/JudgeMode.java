package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.feature.Feature;
import org.firstinspires.ftc.teamcode.opmodes.feature.ServoFeature;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;
import org.firstinspires.ftc.teamcode.util.ThresholdTrigger;

/**
 * Judge mode for evaluating the features of the robot
 */
@TeleOp(name="Judge Mode", group="Judge")
public class JudgeMode extends OpMode {

    private final Tilerunner tilerunner = new Tilerunner();
    private Feature whacker;

    private final ThresholdTrigger glyphPut  = new ThresholdTrigger();
    private final ThresholdTrigger glyphGrab = new ThresholdTrigger();

    @Override
    public void init() {
        tilerunner.init(hardwareMap, telemetry, Tilerunner.OpmodeType.TELEOP);
        whacker = new ServoFeature(tilerunner.jewelWhacker, 0.05);
    }

    private double calculateLiftSpeed(double joystickPower) {
        return (joystickPower * joystickPower) * Math.signum(joystickPower);
    }

    @Override
    public void loop() {
        Twigger.getInstance().update();

        boolean glyphDetected = tilerunner.glyphDetected();
        tilerunner.setLights(glyphDetected, glyphDetected);

        // Claw Motor (Gamepad 1 Triggers)
        double putVal = glyphPut.get(gamepad1.left_trigger);
        if (putVal > 0) {
            tilerunner.ejectGlyph(putVal);
        } else {
            tilerunner.grabGlyph(glyphGrab.get(gamepad1.right_trigger));
        }

        // Lift (Gamepad 2 Left Joystick)
        tilerunner.setLiftPower(calculateLiftSpeed(-gamepad1.left_stick_y));

        // Jewel Whacker (Gamepad 2 Right Joystick)
        whacker.call(-gamepad1.right_stick_y);
    }
}
