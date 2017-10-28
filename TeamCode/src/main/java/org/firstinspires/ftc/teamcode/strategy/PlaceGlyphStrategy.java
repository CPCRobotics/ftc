package org.firstinspires.ftc.teamcode.strategy;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.Tilerunner;

/**
 * Picks up a glyph and moves it to the correct cryptobox column
 */

public class PlaceGlyphStrategy {

    private final Tilerunner tilerunner;
    private final TeamPosition position;
    private final BusyWaitHandler waitHandler;

    public PlaceGlyphStrategy(Tilerunner tilerunner, TeamPosition position, BusyWaitHandler waitHandler) {
        this.position = position;
        this.tilerunner = tilerunner;
        this.waitHandler = waitHandler;
    }

    public void placeGlyph(CryptoboxColumn column) throws InterruptedException {
        moveToCryptoboxColumn(column);
        putDownGlyph();
    }

    /**
     * Measure the difference of how far to go depending
     * on the cryptobox column.
     *
     * Add if on blue team. Subtract if on red team.
     */
    private double cryptoboxOffset(CryptoboxColumn column) {
        switch (column) {
            case LEFT:   return -7.5;
            case CENTER: return 0;
            case RIGHT:
            default:     return 7.5;
        }
    }

    private void moveToCryptoboxColumn(CryptoboxColumn column) throws InterruptedException {
        // Move to the cryptobox depending on the position
        switch (position) {
            case BLUE_A:
                tilerunner.move(waitHandler, 1, 33.5 + cryptoboxOffset(column));
                tilerunner.turn(waitHandler, -1, 90);
                break;
            case RED_A:
                tilerunner.move(waitHandler, 1, -(33.5 - cryptoboxOffset(column)));
                tilerunner.turn(waitHandler, -1, 90);
                break;
            case BLUE_FAR:
                tilerunner.move(waitHandler, 1, 23);
                tilerunner.turn(waitHandler, 1, 90);
                tilerunner.move(waitHandler, 1, 13 + cryptoboxOffset(column));
                tilerunner.turn(waitHandler, -1, 90);
                break;
            case RED_FAR:
                tilerunner.move(waitHandler, 1, -26);
                tilerunner.turn(waitHandler, 1, 90);
                tilerunner.move(waitHandler, 1, 12.5 - cryptoboxOffset(column));
                tilerunner.turn(waitHandler, 1, 90);
                break;
        }
    }

    private void putDownGlyph() throws InterruptedException {
        tilerunner.removeGlyph(waitHandler, 1);
    }

}
