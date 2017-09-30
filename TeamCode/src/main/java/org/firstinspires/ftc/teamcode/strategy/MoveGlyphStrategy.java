package org.firstinspires.ftc.teamcode.strategy;

/**
 * Picks up a glyph and moves it to the correct cryptobox column
 */

public class MoveGlyphStrategy {

    public void pickUpGlyph() {
        moveToGlyph();
        grabGlyph();
        goToCryptobox();
        CryptoboxColumn column = decideCryptoboxColumn();
        moveToCryptoboxColumn(column);
        putDownGlyph();
    }

    private void moveToGlyph() {
        throw new UnsupportedOperationException();
    }

    private void grabGlyph() {
        throw new UnsupportedOperationException();
    }

    private void goToCryptobox() {
        throw new UnsupportedOperationException();
    }

    private CryptoboxColumn decideCryptoboxColumn() {
        throw new UnsupportedOperationException();
    }

    private void moveToCryptoboxColumn(CryptoboxColumn column) {
        throw new UnsupportedOperationException();
    }

    private void putDownGlyph() {
        throw new UnsupportedOperationException();
    }

}
