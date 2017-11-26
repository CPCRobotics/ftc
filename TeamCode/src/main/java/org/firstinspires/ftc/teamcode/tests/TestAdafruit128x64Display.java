package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.hardware.AdafruitMono128x64;

/**
 * Displays different symbols on the HD Display to verify it is working
 */
@Autonomous(name = "Adafruit Mono OLED Display", group = "TestHardware")
@Disabled
public class TestAdafruit128x64Display extends LinearOpMode {

    AdafruitMono128x64 display;
    DigitalChannel resetPin;

    private void prompt(String text) {
        telemetry.addData("showing", text);
        telemetry.update();
        display.display();
        sleep(500);

    }

    @Override public void runOpMode() throws InterruptedException {

        display = hardwareMap.get(AdafruitMono128x64.class, "oled_display");
        resetPin = hardwareMap.get(DigitalChannel.class, "oled_reset");
        display.reset(resetPin);

        AdafruitMono128x64.Graphix graphix = display.getGraphix();
        graphix.drawPixel(4,4,graphix.WHITE);
        graphix.display();

        // wait for the start button to be pressed
        waitForStart();

        if (opModeIsActive()) {
            graphix.drawLine(0,0, 7,0, graphix.WHITE);
            prompt("Red line left to right");
        }
        if (opModeIsActive()) {
            graphix.drawLine(0,0, 7,7, graphix.WHITE);
            prompt("Green diagonol");
        }
        if (opModeIsActive()) {
            graphix.drawLine(0,0, 0,7, graphix.WHITE);
            prompt("Yellow line top to bottom");
        }
        if (opModeIsActive()) {
            graphix.drawRect(0,0, 8,8, graphix.WHITE);
            prompt("Red rectangle");
        }
        if (opModeIsActive()) {
            graphix.fillRect(0,0, 8,8, graphix.WHITE);
            prompt("Green fill");
        }
        if (opModeIsActive()) {
            graphix.clearScreen();
            prompt("Clear");
        }
        if (opModeIsActive()) {
            graphix.drawChar(0,0, '\1');
            prompt("Yellow \\1");
        }
        if (opModeIsActive()) {
            graphix.drawChar(0,0, 'A');
            prompt("Red A");
        }
        if (opModeIsActive()) {
            graphix.setFG(graphix.WHITE);
            graphix.drawChar(0,0, 'B');
            prompt("Red B");
        }
        if (opModeIsActive()) {
            graphix.setFG(graphix.WHITE);
            graphix.drawChar(0,0, 'C');
            prompt("Green C");
        }
        if (opModeIsActive()) {
            graphix.clearScreen();
            graphix.drawCircle(4,4, 3, graphix.WHITE);
            prompt("Red Circle");
        }
        if (opModeIsActive()) {
            graphix.fillCircle(4,4, 3, graphix.WHITE);
            prompt("Green Fill Circle");
        }
        if (opModeIsActive()) {
            graphix.drawChar(2,0, '!');
            prompt("Exclamation !");
        }
        byte [] bitmap = { (byte)0xFF, (byte)0x99, (byte)0x99, (byte)0xFF, (byte)0xFF, (byte)0x99, (byte)0x99, (byte)0xFF };
        if (opModeIsActive()) {
            graphix.drawBitmap(0,0, bitmap, 8, 8);
            prompt("Bitmap");
        }
    }
}
