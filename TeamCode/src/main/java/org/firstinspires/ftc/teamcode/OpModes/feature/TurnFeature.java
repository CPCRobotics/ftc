package org.firstinspires.ftc.teamcode.OpModes.feature;

import org.firstinspires.ftc.teamcode.Tilerunner;

/**
 * Easy Turn the robot
 */
public final class TurnFeature extends Feature {
    private final Tilerunner tilerunner;
    public TurnFeature(Tilerunner tilerunner) {
        this.tilerunner = tilerunner;
    }

    /**
     *
     * @param value Turns right if positive
     */
    @Override
    public boolean call(double value) {
        tilerunner.setMotors(value, -value);
        return (value != 0);
    }
}
