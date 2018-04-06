package org.firstinspires.ftc.teamcode.strategy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.util.EyesightUtil;
import org.firstinspires.ftc.teamcode.Tilerunner;

/**
 * Stitches together all the code for autonomous mode
 */
public class AutonomousDelegator {

    private final TeamPosition position;
    private final Tilerunner tilerunner;
    private final BusyWaitHandler waitHandler;
    private final OpMode opMode;

    public AutonomousDelegator(TeamPosition position, BusyWaitHandler waitHandler, OpMode opMode) throws InterruptedException {
        this.position = position;
        this.tilerunner = new Tilerunner();
        this.waitHandler = waitHandler;
        this.opMode = opMode;

        // Set up stuff
        tilerunner.init(opMode.hardwareMap, opMode.telemetry, Tilerunner.OpmodeType.AUTONOMOUS);
        tilerunner.zeroLift(waitHandler);

        EyesightUtil.init(opMode);
    }

    public void start() throws InterruptedException {
        tilerunner.setLights(false, false);
        CryptoboxColumn column = new PictographPhase(tilerunner, waitHandler).readCryptoboxKey();
        double offset = new JewelTopplerPhase(position, waitHandler, opMode.hardwareMap.appContext, tilerunner).toppleEnemyJewel();
        new PlaceGlyphPhase(tilerunner, position, waitHandler).placeGlyph(column, offset);
    }

}
