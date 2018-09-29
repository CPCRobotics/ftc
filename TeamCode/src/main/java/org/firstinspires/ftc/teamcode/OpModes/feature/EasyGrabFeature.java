package org.firstinspires.ftc.teamcode.OpModes.feature;

import org.firstinspires.ftc.teamcode.Tilerunner;

public class EasyGrabFeature extends Feature {
    private final Tilerunner tilerunner;
    public EasyGrabFeature(Tilerunner tilerunner) {
        this.tilerunner = tilerunner;
    }

    @Override
    public boolean call(double value) {
        if (value <= 0)
            return false;

        tilerunner.setMotors(value, value);
        tilerunner.grabGlyph(1);
        return true;
    }
}
