package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.hardware.AdafruitGraphix;
import org.firstinspires.ftc.teamcode.hardware.AdafruitMono128x64;

/**
 * Displays different symbols on the HD Display to verify it is working
 */
@Autonomous(name = "Adafruit Mono OLED Display", group = "TestHardware")
@Disabled
public class TestAdafruit128x64Display extends LinearOpMode {

    private AdafruitMono128x64 display;

    private void prompt(String text) throws InterruptedException {
        telemetry.addData("showing", text);
        telemetry.update();
        display.display();
        sleep(500);
        if (!opModeIsActive())
            Thread.currentThread().interrupt();
    }

    @Override public void runOpMode() throws InterruptedException {

        display = hardwareMap.get(AdafruitMono128x64.class, "oled_display");
        DigitalChannel resetPin = hardwareMap.get(DigitalChannel.class, "oled_reset");
        display.reset(resetPin);

        AdafruitMono128x64.Graphix graphix = display.getGraphix();
        graphix.drawPixel(4,4, AdafruitGraphix.WHITE);
        graphix.display();

        // wait for the start button to be pressed
        waitForStart();

        graphix.drawLine(0,0, 7,0, AdafruitGraphix.WHITE);
        prompt("Red line left to right");

        graphix.drawLine(0,0, 7,7, AdafruitGraphix.WHITE);
        prompt("Green diagonol");

        graphix.drawLine(0,0, 0,7, AdafruitGraphix.WHITE);
        prompt("Yellow line top to bottom");

        graphix.drawRect(0,0, 8,8, AdafruitGraphix.WHITE);
        prompt("Red rectangle");

        graphix.fillRect(0,0, 8,8, AdafruitGraphix.WHITE);
        prompt("Green fill");

        graphix.clearScreen();
        prompt("Clear");

        graphix.drawChar(0,0, '\1');
        prompt("Yellow \\1");

        graphix.drawChar(0,0, 'A');
        prompt("Red A");

        graphix.setFG(AdafruitGraphix.WHITE);
        graphix.drawChar(0,0, 'B');
        prompt("Red B");

        graphix.setFG(AdafruitGraphix.WHITE);
        graphix.drawChar(0,0, 'C');
        prompt("Green C");

        graphix.clearScreen();
        graphix.drawCircle(4,4, 3, AdafruitGraphix.WHITE);
        prompt("Red Circle");

        graphix.fillCircle(4,4, 3, AdafruitGraphix.WHITE);
        prompt("Green Fill Circle");

        graphix.drawChar(2,0, '!');
        prompt("Exclamation !");

        byte [] bitmap = { (byte)0xFF, (byte)0x99, (byte)0x99, (byte)0xFF, (byte)0xFF, (byte)0x99, (byte)0x99, (byte)0xFF };

        graphix.drawBitmap(0,0, bitmap, 8, 8);
        prompt("Bitmap");
    }
}
