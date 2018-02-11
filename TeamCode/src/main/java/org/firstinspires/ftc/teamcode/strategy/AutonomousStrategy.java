package org.firstinspires.ftc.teamcode.strategy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.util.EyesightUtil;
import org.firstinspires.ftc.teamcode.Tilerunner;

/**
 * Stitches together all the code for autonomous mode
 */
public class AutonomousStrategy {

    private final TeamPosition position;
    private final Tilerunner tilerunner;
    private final BusyWaitHandler waitHandler;
    private final OpMode opMode;

    public AutonomousStrategy(TeamPosition position, BusyWaitHandler waitHandler, OpMode opMode) throws InterruptedException {
        this.position = position;
        this.tilerunner = new Tilerunner();
        this.waitHandler = waitHandler;
        this.opMode = opMode;

        // Set up stuff
        tilerunner.init(opMode.hardwareMap, opMode.telemetry);
        tilerunner.zeroLift(waitHandler, false);

        EyesightUtil.init(opMode);
    }

    public void start() throws InterruptedException {
        CryptoboxColumn column = new PictographStrategy(waitHandler, tilerunner).readCryptoboxKey();
        double offset = new JewelTopplerStrategy(position, waitHandler, opMode.hardwareMap.appContext, tilerunner).toppleEnemyJewel();
        new PlaceGlyphStrategy(tilerunner, position, waitHandler).placeGlyph(column, offset);
    }

}
