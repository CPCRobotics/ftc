package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.AdafruitBiColorMatrix;
import org.firstinspires.ftc.teamcode.hardware.AdafruitMono128x64;
import org.firstinspires.ftc.teamcode.hardware.AdafruitVCLN4010;

@Autonomous(name = "Adafruit BiColor Display", group = "Test")
public class TestAdafruitDisplay extends LinearOpMode {

    AdafruitBiColorMatrix display;

    private void prompt(String text) {
        telemetry.addData("showing", text);
        telemetry.update();
        display.display();
        sleep(500);

    }

    @Override public void runOpMode() {

        display = hardwareMap.get(AdafruitBiColorMatrix.class, "led_matrix");
        display.setRotation(1);
        AdafruitBiColorMatrix.Graphix graphix = display.getGraphix();
        graphix.drawPixel(4,4,graphix.YELLOW);
        graphix.display();

        // wait for the start button to be pressed
        waitForStart();

        if (opModeIsActive()) {
            graphix.drawLine(0,0, 7,0, graphix.RED);
            prompt("Red line left to right");
        }
        if (opModeIsActive()) {
            graphix.drawLine(0,0, 7,7, graphix.GREEN);
            prompt("Green diagonol");
        }
        if (opModeIsActive()) {
            graphix.drawLine(0,0, 0,7, graphix.YELLOW);
            prompt("Yellow line top to bottom");
        }
        if (opModeIsActive()) {
            graphix.drawRect(0,0, 8,8, graphix.RED);
            prompt("Red rectangle");
        }
        if (opModeIsActive()) {
            graphix.fillRect(0,0, 8,8, graphix.GREEN);
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
            graphix.setFG(graphix.RED);
            graphix.drawChar(0,0, 'B');
            prompt("Red B");
        }
        if (opModeIsActive()) {
            graphix.setFG(graphix.GREEN);
            graphix.drawChar(0,0, 'C');
            prompt("Green C");
        }
        if (opModeIsActive()) {
            graphix.clearScreen();
            graphix.drawCircle(4,4, 3, graphix.RED);
            prompt("Red Circle");
        }
        if (opModeIsActive()) {
            graphix.fillCircle(4,4, 3, graphix.GREEN);
            prompt("Green Fill Circle");
        }
        if (opModeIsActive()) {
            graphix.drawChar(2,0, '!');
            prompt("Exclamation !");
        }
        byte [] bitmap = { (byte)0xFF, (byte)0x99, (byte)0x99, (byte)0xFF, (byte)0xFF, (byte)0x99, (byte)0x99, (byte)0xFF };
        if (opModeIsActive()) {
            graphix.setFG(graphix.GREEN);
            graphix.setBG(graphix.YELLOW);
            graphix.drawBitmap(0,0, bitmap, 8, 8);
            prompt("Bitmap");
        }
    }
}
