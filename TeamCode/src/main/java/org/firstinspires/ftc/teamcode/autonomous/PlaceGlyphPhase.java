package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.Tilerunner;

/**
 * Picks up a glyph and moves it to the correct cryptobox column
 */

public class PlaceGlyphPhase {

    private final Tilerunner tilerunner;
    private final TeamPosition position;
    private final BusyWaitHandler waitHandler;

    private final double DEF_SPEED = .5;

    public static final double COLUMNBOX_WIDTH = 7.25;

    public PlaceGlyphPhase(Tilerunner tilerunner, TeamPosition position, BusyWaitHandler waitHandler) {
        this.position = position;
        this.tilerunner = tilerunner;
        this.waitHandler = waitHandler;
    }

    public void placeGlyph(CryptoboxColumn column, double offset) throws InterruptedException {
        moveToCryptoboxColumn(column, offset);
        tilerunner.ejectGlyph(waitHandler);
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

    private void moveToCryptoboxColumn(CryptoboxColumn column, double offset) throws InterruptedException {
        final double TURN_SPEED = .5;
        final double BSTONE_SPEED = .4; // go slower when getting off of the balance stone
        // Move to the cryptobox depending on the position
        switch (position) {
            case BLUE_A:
                tilerunner.moveInches(waitHandler, BSTONE_SPEED, 33.5 + cryptoboxOffset(column) - offset);
                tilerunner.turn(waitHandler, TURN_SPEED, -90);
                tilerunner.moveInches(waitHandler, DEF_SPEED, 9);
                break;
            case RED_A:
                tilerunner.moveInches(waitHandler, BSTONE_SPEED, -(30.25 - cryptoboxOffset(column)) - offset);
                tilerunner.turn(waitHandler, TURN_SPEED, -90);
                tilerunner.moveInches(waitHandler, DEF_SPEED, 9);
                break;
            case BLUE_FAR:
                tilerunner.moveInches(waitHandler, BSTONE_SPEED, 26 - offset);
                tilerunner.turn(waitHandler, TURN_SPEED, 90);
                tilerunner.moveInches(waitHandler, DEF_SPEED, 13 + cryptoboxOffset(column));
                tilerunner.turn(waitHandler, TURN_SPEED, -90);
                tilerunner.moveInches(waitHandler, DEF_SPEED, 5);
                break;
            case RED_FAR:
                tilerunner.moveInches(waitHandler, BSTONE_SPEED, -28 - offset);
                tilerunner.turn(waitHandler, TURN_SPEED, 90);
                tilerunner.moveInches(waitHandler, DEF_SPEED, 13 - cryptoboxOffset(column));
                tilerunner.turn(waitHandler, TURN_SPEED, 90);
                tilerunner.moveInches(waitHandler, DEF_SPEED, 3);
                break;
        }
    }

    private void prepareForTele() throws InterruptedException {
        final double inches = 10;
        final double PUSH_GLYPH_IN = 4;
        final double BACK_UP_IN = 10;
        switch (position) {
            case RED_A:
            case BLUE_A:
                tilerunner.moveInches(waitHandler, DEF_SPEED, -inches);
                tilerunner.grabGlyph(0);
                tilerunner.turn(waitHandler, DEF_SPEED, 180);
                tilerunner.moveTimed(waitHandler, -DEF_SPEED / 2, 1.5);

                tilerunner.moveInches(waitHandler, DEF_SPEED, BACK_UP_IN);
                break;
            case RED_FAR:
            case BLUE_FAR:
                tilerunner.moveInches(waitHandler, DEF_SPEED, -BACK_UP_IN);
        }
    }

}
