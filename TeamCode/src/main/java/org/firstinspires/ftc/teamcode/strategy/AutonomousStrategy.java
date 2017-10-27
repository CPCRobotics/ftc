package org.firstinspires.ftc.teamcode.strategy;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.EyesightUtil;
import org.firstinspires.ftc.teamcode.Tilerunner;

/**
 * Created by samuel on 10/20/17.
 */

public class AutonomousStrategy {

    private final TeamPosition position;
    private final Tilerunner tilerunner;
    private final BusyWaitHandler waitHandler;
    private final OpMode opMode;

    public AutonomousStrategy(TeamPosition position, BusyWaitHandler waitHandler, OpMode opMode) {
        this.position = position;
        this.tilerunner = new Tilerunner();
        this.waitHandler = waitHandler;
        this.opMode = opMode;

        // Set up stuff
        tilerunner.init(opMode.hardwareMap, opMode.telemetry);
        EyesightUtil.init(opMode);
    }

    public void start() throws InterruptedException {
        //new JewelTopplerStrategy(position, waitHandler, opMode.hardwareMap.appContext, tilerunner).toppleEnemyJewel();
        //CryptoboxColumn column = new PictographStrategy(waitHandler, tilerunner).readCryptoboxKey();
        new PlaceGlyphStrategy(tilerunner, position, waitHandler).placeGlyph(CryptoboxColumn.CENTER);
    }

}
