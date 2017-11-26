package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.AdafruitADPS9960;


/**
 * Telemetries data from the range sensor
 */
@Autonomous(name = "Adafruit range sensor", group = "TestHardware")
public class TestAdafruitRangeSensor extends LinearOpMode {

    private AdafruitADPS9960 rangeSensor;

    @Override public void runOpMode() {

        // get a reference to our compass
        rangeSensor = hardwareMap.get(AdafruitADPS9960.class, "range");

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
