package org.firstinspires.ftc.teamcode.strategy;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.EyesightUtil;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

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

        ElapsedTime time = new ElapsedTime();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && waitHandler.isActive() && time.seconds() < 5)
            vuMark = EyesightUtil.getPictograph();

        CryptoboxColumn column = CryptoboxColumn.fromVuMark(vuMark);

        Twigger.getInstance().sendOnce("Detected Pictogram: " + column.name());
        column.displayPosition(tilerunner);

        EyesightUtil.stop();


        return CryptoboxColumn.fromVuMark(vuMark);
    }
}
