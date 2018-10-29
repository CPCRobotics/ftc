package org.firstinspires.ftc.teamcode.Autonomous;

/**
 * Represents different starting positions
 */
public enum TeamPosition
{
    RED_A(true, true),
    RED_FAR(true, false),
    BLUE_A(false, true),
    BLUE_FAR(false, false);

    private final boolean red;
    private final boolean audience;
    TeamPosition(boolean red, boolean audience)
    {
        this.red = red;
        this.audience = audience;
    }

    public boolean isRed() { return red; }
    public boolean isBlue() { return !red; }
    public boolean isAudience() {
        return audience;
    }
}
