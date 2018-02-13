package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.twigger.Twigger;

/**
 * Finding a way to see if a gamepad can be detected to be paired
 */
@TeleOp(name="Test Gamepad Timestamp", group="TestSoftware")
public class TestGamepadTimestamp extends OpMode {

    private ElapsedTime updateTime = new ElapsedTime();
    @Override
    public void init() {
        Twigger.getInstance().init(telemetry);
        updateTime.reset();
    }

    @Override
    public void loop() {
        if (updateTime.milliseconds() > 250) {
            Twigger.getInstance().addData("timestamp", gamepad1.timestamp)
                    .update();
            updateTime.reset();
        }
    }
}
