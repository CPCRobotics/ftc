package org.firstinspires.ftc.teamcode.strategy;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Picks up a glyph and moves it to the correct cryptobox column
 */

public class MoveGlyphStrategy {

    public void pickUpGlyph() {
        // If pictograph isn't read yet, read it
        if (PictographStrategy.getCryptoboxKey() == CryptoboxColumn.UNKNOWN)
            PictographStrategy.readCryptoboxKey();

        moveToGlyph();
        grabGlyph();
        goToCryptobox();
        moveToCryptoboxColumn(PictographStrategy.getCryptoboxKey());
        putDownGlyph();
    }

    private void moveToGlyph() {
        turnToGlyphPit();
        moveToGlyphPit();
        Position pos = findNearestGlyph();
        goToPosition(pos);
    }

    // Turn from jewels to look at glyph pit
    private void turnToGlyphPit() {
        throw new UnsupportedOperationException();
    }

    // Move from jewels to glyph pit
    private void moveToGlyphPit() {
        throw new UnsupportedOperationException();
    }

    // Find nearest glyph from robot position
    private Position findNearestGlyph() {
        throw new UnsupportedOperationException();
    }

    private void goToPosition(Position pos) {
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
