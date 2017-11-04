package org.firstinspires.ftc.teamcode.strategy;

/**
 * Created by samuel on 10/27/17.
 */

public enum TeamPosition {
    RED_A(true, true),
    RED_FAR(true, false),
    BLUE_A(false, true),
    BLUE_FAR(false, false),
    RESET_LIFT(false, false);

    private boolean red;
    private boolean nearAudience;
    TeamPosition(boolean red, boolean nearAudience) {
        this.red = red;
        this.nearAudience = nearAudience;
    }

    public boolean isRed() { return red; }
    public boolean isBlue() { return !red; }
    public boolean isNearAudience() { return nearAudience; }
    public boolean isNearDrivers() { return !nearAudience; }
}
