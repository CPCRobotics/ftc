package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.AdafruitBiColorMatrix;
import org.firstinspires.ftc.teamcode.hardware.AdafruitGraphix;

/**
 * Displays different symbols on the display to verify it is working
 */
@Autonomous(name = "Adafruit BiColor Display", group = "TestHardware")
public class TestAdafruitDisplay extends LinearOpMode {

    private AdafruitBiColorMatrix display;

    private void prompt(String text) {
        telemetry.addData("showing", text);
        telemetry.update();
        display.display();
        sleep(500);
        if (!opModeIsActive())
            Thread.currentThread().interrupt();
    }

    @Override public void runOpMode() {

        display = hardwareMap.get(AdafruitBiColorMatrix.class, "display");
        display.setRotation(3);
        AdafruitBiColorMatrix.Graphix graphix = display.getGraphix();
        graphix.drawPixel(4,4, AdafruitGraphix.YELLOW);
        graphix.display();

        // wait for the start button to be pressed
        waitForStart();


        graphix.drawLine(0,0, 7,0, AdafruitGraphix.RED);
        prompt("Red line left to right");

        graphix.drawLine(0,0, 7,7, AdafruitGraphix.GREEN);
        prompt("Green diagonol");

        graphix.drawLine(0,0, 0,7, AdafruitGraphix.YELLOW);
        prompt("Yellow line top to bottom");

        graphix.drawRect(0,0, 8,8, AdafruitGraphix.RED);
        prompt("Red rectangle");

        graphix.fillRect(0,0, 8,8, AdafruitGraphix.GREEN);
        prompt("Green fill");

        graphix.clearScreen();
        prompt("Clear");

        graphix.drawChar(0,0, '\1');
        prompt("Yellow \\1");

        graphix.drawChar(0,0, 'A');
        prompt("Red A");

        graphix.setFG(AdafruitGraphix.RED);
        graphix.drawChar(0,0, 'B');
        prompt("Red B");

        graphix.setFG(AdafruitGraphix.GREEN);
        graphix.drawChar(0,0, 'C');
        prompt("Green C");

        graphix.clearScreen();
        graphix.drawCircle(4,4, 3, AdafruitGraphix.RED);
        prompt("Red Circle");

        graphix.fillCircle(4,4, 3, AdafruitGraphix.GREEN);
        prompt("Green Fill Circle");

        graphix.drawChar(2,0, '!');
        prompt("Exclamation !");

        byte [] bitmap = { (byte)0xFF, (byte)0x99, (byte)0x99, (byte)0xFF, (byte)0xFF, (byte)0x99, (byte)0x99, (byte)0xFF };

        graphix.setFG(AdafruitGraphix.GREEN);
        graphix.setBG(AdafruitGraphix.YELLOW);
        graphix.drawBitmap(0,0, bitmap, 8, 8);
        prompt("Bitmap");
    }
}
