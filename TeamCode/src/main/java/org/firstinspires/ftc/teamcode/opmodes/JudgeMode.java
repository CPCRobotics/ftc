package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.GameControls;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.strategy.CryptoboxColumn;
import org.firstinspires.ftc.teamcode.strategy.JewelDirection;
import org.firstinspires.ftc.teamcode.twigger.Twigger;
import org.firstinspires.ftc.teamcode.util.ListCyclicalIterator;

import java.util.Arrays;

/**
 * Judge mode for evaluating the features of the robot
 */
@TeleOp(name="Judge Mode", group="Judge")
public class JudgeMode extends OpMode {

    private final GameControls gameControls = new GameControls(this,
            GameControls.GameMode.JUDGING);

    private Tilerunner tilerunner = new Tilerunner();
    private double whackerPosition = 0;
    private static final double WHACKER_CONTROL_SPEED = 0.05;

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
        tilerunner.init(hardwareMap, telemetry, true);
        whackerPosition = tilerunner.jewelWhacker.getPosition();
    }

    private double calculateLiftSpeed(double joystickPower) {
        return (joystickPower * joystickPower) * Math.signum(joystickPower);
    }

    private static double withinRange(double min, double max, double x) {
        return Math.min(max, Math.max(min, x));
    }

    @Override
    public void loop() {
        Twigger.getInstance().update();

        // 8x8 Display & Previews
        if (gameControls.getToggleDisplay()) {
            liveDisplay = !liveDisplay;
            if (!liveDisplay)
                previews.current().preview();
        }

        if (gameControls.getCyclePrev()) {
            liveDisplay = false;
            previews.prev().preview();
        }

        if (gameControls.getCycleNext()) {
            liveDisplay = false;
            previews.next().preview();
        }

        if (liveDisplay)
            tilerunner.displayStatus();

        // Claw Motor (Gamepad 1 Triggers)
        if (gameControls.getGlyphEjectPower() > 0) {
            tilerunner.ejectGlyph(gameControls.getGlyphEjectPower());
        } else {
            tilerunner.grabGlyph(gameControls.getGlyphGrabPower());
        }

        // Lift (Gamepad 2 Left Joystick)
        tilerunner.setLiftPower(calculateLiftSpeed(gameControls.getLiftDrive()));

        // Jewel Whacker (Gamepad 2 Right Joystick)
        whackerPosition += WHACKER_CONTROL_SPEED * gameControls.getJewelWhackerDrive();
        whackerPosition = withinRange(0, 1, whackerPosition);
        tilerunner.jewelWhacker.setPosition(whackerPosition);

    }
}
