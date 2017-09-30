package org.firstinspires.ftc.teamcode.strategy;

import org.firstinspires.ftc.teamcode.Tilerunner;

/**
 * Finds what Cryptobox Key to go to
 */
public class PictographStrategy {

    private Tilerunner tileRunner;

    public PictographStrategy(Tilerunner tileRunner) {
        this.tileRunner = tileRunner;
    }

    public CryptoboxColumn readCryptoboxKey() {
        goToPictogram();
        readPictogram();
        return getKeyFromPictogram();
    }

    private void goToPictogram() {
        throw new UnsupportedOperationException();
    }

    private void readPictogram() {
        throw new UnsupportedOperationException();
    }

    private CryptoboxColumn getKeyFromPictogram() {
        throw new UnsupportedOperationException();
    }
}
