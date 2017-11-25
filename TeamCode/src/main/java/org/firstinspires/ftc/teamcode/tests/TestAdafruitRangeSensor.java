package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.AdafruitADPS9960;
import org.firstinspires.ftc.teamcode.hardware.AdafruitBiColorMatrix;
import org.firstinspires.ftc.teamcode.hardware.AdafruitVCLN4010;

@Autonomous(name = "Adafruit range sensor", group = "Test")
public class TestAdafruitRangeSensor extends LinearOpMode {

    AdafruitADPS9960 rangeSensor;

    @Override public void runOpMode() {

        // get a reference to our compass
        rangeSensor = hardwareMap.get(AdafruitADPS9960.class, "sensor_range");

        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("raw optical", rangeSensor.readProximityData());
            telemetry.addData("mm", "%.2f mm", rangeSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("ambient", "%d", rangeSensor.readAmbientLightLevel());
            telemetry.addData("color", "(%d, %d, %d)",
                    rangeSensor.readRedData(),
                    rangeSensor.readGreenData(),
                    rangeSensor.readBlueData());
            telemetry.update();
            sleep(1);
        }
    }
}
