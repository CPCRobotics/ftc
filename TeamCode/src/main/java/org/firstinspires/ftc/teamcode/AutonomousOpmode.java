package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        tilerunner.init(hardwareMap, telemetry, true);

        initAutonomous();

        waitForStart();

        startAutonomous();
    }

    protected void initAutonomous() throws InterruptedException {}

    protected abstract void startAutonomous() throws InterruptedException;

    @Override
    public boolean isActive() {
        return false;
    }
}
