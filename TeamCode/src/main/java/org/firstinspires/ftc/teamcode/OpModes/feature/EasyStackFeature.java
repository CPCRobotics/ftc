package org.firstinspires.ftc.teamcode.OpModes.feature;

import org.firstinspires.ftc.teamcode.Tilerunner;

/**
 * Stacks one glyph on another
 */
public class EasyStackFeature extends Feature {
    private final Tilerunner tilerunner;
    private boolean lastActive = false;

    public EasyStackFeature(Tilerunner tilerunner) {
        this.tilerunner = tilerunner;
    }

    @Override
    public boolean call(double value) {
        if (value <= 0) {
            // Bring the lift down if it's been going for over a second
            if (lastActive)
                tilerunner.changeLiftPosition(Tilerunner.CryptoboxRow.LOWEST);

            return (lastActive = false);
        }

        tilerunner.ejectGlyph(1, false);
        tilerunner.motorPair.setPower(-0.4);

        return (lastActive = true);
    }
}
