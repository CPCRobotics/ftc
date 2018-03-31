package org.firstinspires.ftc.teamcode.util;

/**
 * Toggle button by depression.
 *
 * When a button is pushed, it only returns "true" once until it's released & depressed again.
 */
public class DepressedButton {
    private boolean lastPressed = false;

    public boolean get(boolean val) {
        if (!val)
            return (lastPressed = false);

        return !lastPressed && (lastPressed = true);
    }
}
