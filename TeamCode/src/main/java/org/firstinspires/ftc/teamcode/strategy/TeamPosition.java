package org.firstinspires.ftc.teamcode.strategy;

/**
 * Represents different starting balancing stones
 */
public enum TeamPosition {
    RED_A(true),
    RED_FAR(true),
    BLUE_A(false),
    BLUE_FAR(false);

    private final boolean red;
    TeamPosition(boolean red) {
        this.red = red;
    }

    public boolean isRed() { return red; }
    public boolean isBlue() { return !red; }
}
