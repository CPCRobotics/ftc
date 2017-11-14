package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.AdafruitVCLN4010;

@Autonomous(name = "Sensor: Adafruit range sensor", group = "Test")
public class TestAdafruitRangeSensor extends LinearOpMode {

    AdafruitVCLN4010 rangeSensor;

    @Override public void runOpMode() {

        // get a reference to our compass
        rangeSensor = hardwareMap.get(AdafruitVCLN4010.class, "sensor_range");

        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("raw optical", rangeSensor.getRawLightDetected());
            telemetry.addData("mm", "%.2f mm", rangeSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
