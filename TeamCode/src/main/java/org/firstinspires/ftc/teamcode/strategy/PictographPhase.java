package org.firstinspires.ftc.teamcode.strategy;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.util.EyesightUtil;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Finds what Cryptobox Key to go to
 */
@SuppressWarnings("WeakerAccess")
public class PictographPhase {

    private final BusyWaitHandler waitHandler;
    private final Tilerunner tilerunner;

    public PictographPhase(Tilerunner tilerunner, BusyWaitHandler waitHandler) {
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

        EyesightUtil.stop();

        boolean pictographDetected = (vuMark != RelicRecoveryVuMark.UNKNOWN);
        tilerunner.setRed(pictographDetected);
        
        return CryptoboxColumn.fromVuMark(vuMark);
    }
}
