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

        double startHeading = tilerunner.getHeading();

        ElapsedTime time = new ElapsedTime();

        tilerunner.leftMotor.setPower(-Tilerunner.MOTOR_DEADZONE-0.15);
        tilerunner.rightMotor.setPower(Tilerunner.MOTOR_DEADZONE+0.15);

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && waitHandler.isActive() && time.seconds() < 5)
            vuMark = EyesightUtil.getPictograph();
        tilerunner.motorPair.setPower(0);

        Twigger.getInstance().sendOnce("Detected Pictogram: " + vuMark.name());

        double endHeading = tilerunner.getHeading();
        // Go to the right the number of degrees you went plus 90°
        tilerunner.turn(waitHandler, 1,
                Tilerunner.Direction.COUNTERCLOCKWISE.distanceDegrees(startHeading, endHeading));



        tilerunner.move(waitHandler, 1, 6.5);
        return CryptoboxColumn.fromVuMark(vuMark);
    }
}
