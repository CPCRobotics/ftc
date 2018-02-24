package org.firstinspires.ftc.teamcode.opmodes.feature;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tilerunner;

public class EasyPutFeature extends Feature {
    private final Tilerunner tilerunner;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean lastActive = false;

    public EasyPutFeature(Tilerunner tilerunner) {
        this.tilerunner = tilerunner;
    }

    @Override
    public boolean call(double value) {
        boolean active = (value > 0);

        if (active) {
            if (!lastActive)
                timer.reset();

            tilerunner.ejectGlyph(1);
            if (timer.seconds() > 1) {
                tilerunner.setMotors(-0.75);
            } else {
                tilerunner.setMotors(0);
            }
        }

        return (lastActive = active);
    }
}
