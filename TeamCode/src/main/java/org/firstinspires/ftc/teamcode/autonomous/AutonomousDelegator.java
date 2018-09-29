package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private final ElapsedTime timer = new ElapsedTime();

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
        timer.reset();
        tilerunner.setLights(false, false);
        CryptoboxColumn column = new PictographPhase(tilerunner, waitHandler).readCryptoboxKey();
        double offset = new JewelTopplerPhase(position, waitHandler, opMode.hardwareMap.appContext, tilerunner).toppleEnemyJewel();

        final PlaceGlyphPhase place = new PlaceGlyphPhase(tilerunner, position, waitHandler);
        place.placeGlyph(column, offset);

        // Grab an extra glyph only if the situation is right
        if (position.isAudience() && 30 - timer.seconds() > 10)
            new ExtraGlyphPhase(tilerunner, waitHandler).grabExtraGlyph();
    }

}
