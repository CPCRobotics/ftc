package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.AdafruitBiColorMatrix;
import org.firstinspires.ftc.teamcode.hardware.AdafruitVCLN4010;

@Autonomous(name = "Sensor: Adafruit range sensor", group = "Test")
public class TestAdafruitRangeSensor extends LinearOpMode {

    AdafruitVCLN4010 rangeSensor;
    AdafruitBiColorMatrix display;

    @Override public void runOpMode() {

        // get a reference to our compass
        rangeSensor = hardwareMap.get(AdafruitVCLN4010.class, "sensor_range");
        display = hardwareMap.get(AdafruitBiColorMatrix.class, "led_matrix");
        AdafruitBiColorMatrix.Graphix graphix = display.getGraphix();
        graphix.fillScreen(graphix.YELLOW);
        graphix.display();

        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("raw optical", rangeSensor.getRawLightDetected());
            telemetry.addData("mm", "%.2f mm", rangeSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
            graphix.fillScreen(graphix.GREEN);
            graphix.fillRect(0,0, 8, (int)rangeSensor.getDistance(DistanceUnit.MM), graphix.RED);
            graphix.display();

            sleep(1);
        }
    }
}
