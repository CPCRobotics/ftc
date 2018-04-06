package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;

/**
 * Go to glyph pit for extra glyph and bring it back
 */
public class ExtraGlyphPhase {
    private final Tilerunner tilerunner;
    private final BusyWaitHandler busyWaitHandler;

    public ExtraGlyphPhase(Tilerunner tilerunner, BusyWaitHandler busyWaitHandler) {
        this.tilerunner = tilerunner;
        this.busyWaitHandler = busyWaitHandler;
    }

    public void grabExtraGlyph() throws InterruptedException {
        double distanceWent = tilerunner.moveToGlyph(busyWaitHandler, 1, 48);
        tilerunner.moveInches(busyWaitHandler, 1, -distanceWent);
    }
}
