package org.firstinspires.ftc.teamcode.strategy;

import org.firstinspires.ftc.teamcode.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.Tilerunner;

/**
 * Picks up a glyph and moves it to the correct cryptobox column
 */

public class PlaceGlyphStrategy {

    private final Tilerunner tilerunner;
    private final TeamPosition position;
    private final BusyWaitHandler waitHandler;

    public static final double COLUMNBOX_WIDTH = 7.25;

    public PlaceGlyphStrategy(Tilerunner tilerunner, TeamPosition position, BusyWaitHandler waitHandler) {
        this.position = position;
        this.tilerunner = tilerunner;
        this.waitHandler = waitHandler;
    }

    public void placeGlyph(CryptoboxColumn column) throws InterruptedException {
        moveToCryptoboxColumn(column);
        tilerunner.ejectGlyph(waitHandler, true);
        prepareForTele();
    }

    /**
     * Measure the difference of how far to go depending
     * on the cryptobox column.
     *
     * Add if on blue team. Subtract if on red team.
     */
    private double cryptoboxOffset(CryptoboxColumn column) {
        switch (column) {
            case LEFT:   return -COLUMNBOX_WIDTH;
            case RIGHT:  return COLUMNBOX_WIDTH;
            case CENTER:
            default:     return 0;
        }
    }

    private void moveToCryptoboxColumn(CryptoboxColumn column) throws InterruptedException {
        final double TURN_SPEED = 0.75;
        final double BSTONE_SPEED = 0.75; // go slower when getting off of the balance stone
        // Move to the cryptobox depending on the position
        switch (position) {
            case BLUE_A:
                tilerunner.move(waitHandler, BSTONE_SPEED, 33.5 + cryptoboxOffset(column));
                tilerunner.turn(waitHandler, TURN_SPEED, -90);
                tilerunner.move(waitHandler, 1, 9);
                break;
            case RED_A:
                tilerunner.move(waitHandler, BSTONE_SPEED, -(33.5 - cryptoboxOffset(column)));
                tilerunner.turn(waitHandler, TURN_SPEED, -90);
                tilerunner.move(waitHandler, 1, 9);
                break;
            case BLUE_FAR:
                tilerunner.move(waitHandler, BSTONE_SPEED, 26);
                tilerunner.turn(waitHandler, TURN_SPEED, 90);
                tilerunner.move(waitHandler, 1, 13 + cryptoboxOffset(column));
                tilerunner.turn(waitHandler, TURN_SPEED, -90);
                tilerunner.move(waitHandler, 1, 5);
                break;
            case RED_FAR:
                tilerunner.move(waitHandler, BSTONE_SPEED, -28);
                tilerunner.turn(waitHandler, TURN_SPEED, 90);
                tilerunner.move(waitHandler, 1, 13 - cryptoboxOffset(column));
                tilerunner.turn(waitHandler, TURN_SPEED, 90);
                tilerunner.move(waitHandler, 1, 3);
                break;
        }
    }

    private void prepareForTele() throws InterruptedException {
        final double inches = 10;
        switch (position) {
            case RED_A:
            case BLUE_A:
                tilerunner.move(waitHandler, 1, -inches);
                tilerunner.grabGlyph(0);
                tilerunner.turn(waitHandler, 1, 180);
                tilerunner.move(waitHandler, 1, -inches);
                break;
        }
    }

}
