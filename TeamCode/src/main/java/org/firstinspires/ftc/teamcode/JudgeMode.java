package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.strategy.CryptoboxColumn;
import org.firstinspires.ftc.teamcode.strategy.JewelDirection;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Judge mode for evaluating the features of the robot
 */
@TeleOp(name="Judge Mode", group="Judge")
public class JudgeMode extends OpMode {

    private final GameControls gameControls = new GameControls(this);

    private Tilerunner tilerunner = new Tilerunner();
    private double whackerPosition = 0;
    private static final double WHACKER_CONTROL_SPEED = 0.05;

    private boolean liveDisplay = false;
    private JewelDirection previewJewelDirection = JewelDirection.LEFT;
    private CryptoboxColumn previewCryptoboxColumn = CryptoboxColumn.LEFT;

    @Override
    public void init() {
        tilerunner.init(hardwareMap, telemetry, true);
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

        boolean previewChanged = false;
        // 8x8 Display & Previews
        if (gameControls.getToggleDisplay()) {
            liveDisplay = !liveDisplay;
            previewChanged = true;
        }

        if (gameControls.getCycleColumn()) {
            previewCryptoboxColumn = previewCryptoboxColumn.cycleColumn();
            previewChanged = true;
        }

        if (gameControls.getCycleJewel()) {
            previewJewelDirection = previewJewelDirection.cycleDirection();
            previewChanged = true;
        }

        if (liveDisplay) {
            tilerunner.displayStatus();
        } else if (previewChanged) {
            previewCryptoboxColumn.displayPosition(tilerunner);
            previewJewelDirection.displayStatus(tilerunner);
        }

        // Claw Motor (Gamepad 1 Triggers)
        if (gameControls.getGlyphEjectPower() > 0) {
            tilerunner.ejectGlyph(gameControls.getGlyphEjectPower());
        } else {
            tilerunner.grabGlyph(gameControls.getGlyphGrabPower());
        }


        // Lift (Gamepad 2 Left Joystick)
        tilerunner.setLiftPower(calculateLiftSpeed(gameControls.getLiftDrive()));

        // Easy Lift
        // Wait for gamepad2 to be paired for 2 seconds.
        // Since pairing the gamepad requires pushing the B button,
        // it might interfere with the EasyLift functions.
            if (gameControls.getEasyLiftUp())
                tilerunner.changeLiftPosition(true);
            else if (gameControls.getEasyLiftDown())
                tilerunner.changeLiftPosition(false);

            Tilerunner.CryptoboxRow row = gameControls.getEasyLiftPosition();
            if (row != null)
                tilerunner.changeLiftPosition(row);


        // Jewel Whacker (Gamepad 2 Right Joystick)
        whackerPosition += WHACKER_CONTROL_SPEED * gameControls.getJewelWhackerDrive();
        whackerPosition = withinRange(0, 1, whackerPosition);
        tilerunner.jewelWhacker.setPosition(whackerPosition);

    }
}
