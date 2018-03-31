package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Make autonomous opmodes more easily
 */
public abstract class AutonomousOpmode extends LinearOpMode implements BusyWaitHandler {
    protected Tilerunner tilerunner;

    @Override
    public void runOpMode() throws InterruptedException {
        Twigger.getInstance().init(telemetry);
        tilerunner = new Tilerunner();
        tilerunner.init(hardwareMap, telemetry, Tilerunner.OpmodeType.AUTONOMOUS);

        initAutonomous();

        waitForStart();

        startAutonomous();
    }

    protected void initAutonomous() throws InterruptedException {}

    protected abstract void startAutonomous() throws InterruptedException;

    @Override
    public boolean isActive() {
        Twigger.getInstance().update();
        tilerunner.compass.updateCompass();
        return opModeIsActive();
    }
}
