package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import cpc.robotics.logging.NetLogContext;

/**
 * Tests the NetLogContext logger
 */
@Autonomous(name="Test Logger", group="TestSoftware")
public class TestLogger extends LinearOpMode {

    // This provides a context and ensures logging server is started
    // Server will start when this autonomous opmode is initialized.
    private final NetLogContext log = new NetLogContext(LinearOpMode.class);

    public static class ImmutableData {
        public final String bee;
        public final String boo;
        public ImmutableData(String bee, String boo) {
            this.bee = bee;
            this.boo = boo;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        log.write("Before Start");
        log.updateTelemetry("foo", "bar");
        log.updateTelemetry("bing", 1234);
        log.updateTelemetry("buzz", new ImmutableData("a", "b"));
        waitForStart();
        log.write("Started");
    }
}
