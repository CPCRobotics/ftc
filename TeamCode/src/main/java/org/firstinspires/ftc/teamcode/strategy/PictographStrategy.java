package org.firstinspires.ftc.teamcode.strategy;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.EyesightUtil;
import org.firstinspires.ftc.teamcode.Tilerunner;

/**
 * Finds what Cryptobox Key to go to
 */
@SuppressWarnings("WeakerAccess")
public class PictographStrategy {

    private final BusyWaitHandler waitHandler;
    private final Tilerunner tilerunner;

    private static final double INTERVAL_ADJUST_DEGREES = 10;

    public PictographStrategy(BusyWaitHandler waitHandler, Tilerunner tilerunner) {
        this.waitHandler = waitHandler;
        this.tilerunner = tilerunner;
    }

    public CryptoboxColumn readCryptoboxKey() throws InterruptedException {
        EyesightUtil.start();

        tilerunner.move(waitHandler, -1, 6.5);
        RelicRecoveryVuMark vuMark = EyesightUtil.getPictograph();
        tilerunner.move(waitHandler, 1, 6.5);
        return CryptoboxColumn.fromVuMark(vuMark);
    }
}
