package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Centralize game controls in one class
 */
public class GameControls {
    enum GameMode {
        COMPETITION,
        JUDGING;
    }

    private final OpMode opMode;
    private final GameMode gameMode;
    private static final double TRIGGER_THRESHOLD = 0.2;

    // Judging depress keys
    private boolean toggleDisplayDepressed = false;
    private boolean cycleColumnDepressed = false;
    private boolean cycleJewelDepressed = false;

    public GameControls(OpMode opMode, GameMode gameMode) {
        this.gameMode = gameMode;
        this.opMode = opMode;
    }

    public GameControls(OpMode opMode) {
        this(opMode, GameMode.COMPETITION);
    }

    public boolean getEasyPutGlyph() {
        return opMode.gamepad1.left_bumper;
    }

    public boolean getEasyGrabGlyph() {
        return opMode.gamepad1.right_bumper;
    }

    public boolean getTurningLeftButton() {
        return opMode.gamepad1.x;
    }

    public boolean getTurningRightButton() {
        return opMode.gamepad1.b;
    }

    public double getGlyphEjectPower() {
        if (opMode.gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            return opMode.gamepad1.left_trigger;
        } else {
            return 0;
        }
    }

    public double getGlyphGrabPower() {
        if (opMode.gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            return opMode.gamepad1.right_trigger;
        } else {
            return 0;
        }
    }

    public double getLeftDrive() {
        return -opMode.gamepad1.left_stick_y;
    }

    public double getRightDrive() {
        return -opMode.gamepad1.right_stick_y;
    }

    public double getLiftDrive() {
        if (gameMode == GameMode.COMPETITION) {
            return -opMode.gamepad2.left_stick_y;
        } else {
            return -opMode.gamepad1.left_stick_y;
        }
    }

    public boolean getEasyLiftUp() {
        return opMode.gamepad2.dpad_up;
    }

    public boolean getEasyLiftDown() {
        return opMode.gamepad2.dpad_down;
    }

    public Tilerunner.CryptoboxRow getEasyLiftPosition() {
        if (opMode.gamepad2.y)
            return Tilerunner.CryptoboxRow.HIGHEST;
        else if (opMode.gamepad2.x)
            return Tilerunner.CryptoboxRow.HIGHER;
        else if (opMode.gamepad2.b)
            return Tilerunner.CryptoboxRow.LOWER;
        else if (opMode.gamepad2.a)
            return Tilerunner.CryptoboxRow.LOWEST;
        else
            return null;
    }

    public double getJewelWhackerDrive() {
        // Gamepad 2 on Competition; Gamepad 1 on Judging
        Gamepad gamepad = gameMode == GameMode.COMPETITION ? opMode.gamepad2 : opMode.gamepad1;

        if (Math.abs(gamepad.right_stick_y) >= TRIGGER_THRESHOLD) {
            return gamepad.right_stick_y;
        } else {
            return 0;
        }
    }

    public boolean getToggleDisplay() {
        if (opMode.gamepad1.dpad_up) {
            return !toggleDisplayDepressed && (toggleDisplayDepressed = true);
        } else {
            return (toggleDisplayDepressed = false);
        }
    }

    public boolean getCycleColumn() {
        if (opMode.gamepad1.dpad_left) {
            return !cycleColumnDepressed && (cycleColumnDepressed = true);
        } else {
            return (cycleColumnDepressed = false);
        }
    }

    public boolean getCycleJewel() {
        if (opMode.gamepad1.dpad_right) {
            return !cycleJewelDepressed && (cycleJewelDepressed = true);
        } else {
            return (cycleJewelDepressed = false);
        }
    }

}
