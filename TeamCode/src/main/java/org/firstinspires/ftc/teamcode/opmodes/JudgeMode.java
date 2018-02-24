package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.feature.Feature;
import org.firstinspires.ftc.teamcode.opmodes.feature.ServoFeature;
import org.firstinspires.ftc.teamcode.util.DepressedButton;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.strategy.CryptoboxColumn;
import org.firstinspires.ftc.teamcode.strategy.JewelDirection;
import org.firstinspires.ftc.teamcode.twigger.Twigger;
import org.firstinspires.ftc.teamcode.util.ListCyclicalIterator;
import org.firstinspires.ftc.teamcode.util.ThresholdTrigger;

import java.util.Arrays;

/**
 * Judge mode for evaluating the features of the robot
 */
@TeleOp(name="Judge Mode", group="Judge")
public class JudgeMode extends OpMode {

    private Tilerunner tilerunner = new Tilerunner();
    private Feature whacker;

    private final DepressedButton previewToggle = new DepressedButton();
    private final DepressedButton previewLeft   = new DepressedButton();
    private final DepressedButton previewRight  = new DepressedButton();
    private final ThresholdTrigger glyphPut  = new ThresholdTrigger();
    private final ThresholdTrigger glyphGrab = new ThresholdTrigger();

    private boolean liveDisplay = false;

    private class DisplayPreview {
        private final JewelDirection direction;
        private final CryptoboxColumn column;

        DisplayPreview(JewelDirection direction, CryptoboxColumn column) {
            this.direction = direction;
            this.column = column;
        }

        void preview() {
            column.displayPosition(tilerunner);
            direction.displayStatus(tilerunner);
        }
    }

    private ListCyclicalIterator<DisplayPreview> previews
            = new ListCyclicalIterator<>(Arrays.asList(
            new DisplayPreview(JewelDirection.LEFT, CryptoboxColumn.CENTER),
            new DisplayPreview(JewelDirection.RIGHT, CryptoboxColumn.RIGHT),
            new DisplayPreview(JewelDirection.UNKNOWN, CryptoboxColumn.UNKNOWN)
    ));

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

        // 8x8 display and previews
        if (previewToggle.get(gamepad1.dpad_up)) {
            liveDisplay = !liveDisplay;
            if (!liveDisplay)
                previews.current().preview();
        }

        if (previewLeft.get(gamepad1.dpad_left)) {
            liveDisplay = false;
            previews.prev().preview();
        }

        if (previewRight.get(gamepad1.dpad_right)) {
            liveDisplay = true;
            previews.next().preview();
        }

        if (liveDisplay)
            tilerunner.displayStatus();

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
