package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;
import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.util.DepressedButton;
import org.firstinspires.ftc.teamcode.util.PIDController;

/**
 * Drives the robot back and forth to test Tilerunner.move().
 *
 * For PID Tuning
 */
@TeleOp(name="Test PID Move", group="TestConcept")
public class TestPIDMove extends LinearOpMode implements BusyWaitHandler {

    private final double DEST_IN = 36;
    private final double POWER = 0.5;

    private double kP = 0.01;
    private double kI = 0.01;
    private double kD = 0.00;

    private double magnitude = -2; // 10¯²
    private int direction = 1; // Raise, lower

    private final DepressedButton kPButton = new DepressedButton();
    private final DepressedButton kIButton = new DepressedButton();
    private final DepressedButton kDButton = new DepressedButton();
    private final DepressedButton kRaiseButton = new DepressedButton();
    private final DepressedButton kLowerButton = new DepressedButton();
    private final DepressedButton kNegButton = new DepressedButton();
    private final DepressedButton kPosButton = new DepressedButton();

    @Override
    public void runOpMode() throws InterruptedException {

        Tilerunner tilerunner = new Tilerunner();
        tilerunner.init(hardwareMap, telemetry, Tilerunner.OpmodeType.AUTONOMOUS);

        waitForStart();

        while (opModeIsActive()) {
            tilerunner.move(this, DEST_IN,
                    new PIDController(-POWER, POWER, kP, kI, kD));
            Thread.sleep(1000);
            tilerunner.move(this, -DEST_IN,
                    new PIDController(-POWER, POWER, kP, kI, kD));
            Thread.sleep(1000);
        }
    }

    @Override
    public boolean isActive() {
        if (kNegButton.get(gamepad1.dpad_left)) direction = -1;
        if (kPosButton.get(gamepad1.dpad_right)) direction = 1;
        if (kRaiseButton.get(gamepad1.dpad_up)) magnitude++;
        if (kLowerButton.get(gamepad1.dpad_down)) magnitude--;

        if (kPButton.get(gamepad1.x)) kP += Math.pow(10, magnitude) * direction;
        if (kIButton.get(gamepad1.y)) kI += Math.pow(10, magnitude) * direction;
        if (kDButton.get(gamepad1.b)) kD += Math.pow(10, magnitude) * direction;

        Twigger.getInstance()
                .addLine("PID")
                    .addData("mag", magnitude)
                    .addData("dir", direction)
                    .addData("kP", kP)
                    .addData("kI", kI)
                    .addData("kD", kD)
                .done()
                .update();
        return opModeIsActive();
    }
}
