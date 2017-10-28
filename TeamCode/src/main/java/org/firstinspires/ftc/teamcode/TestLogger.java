package org.firstinspires.ftc.teamcode;

import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.lasarobotics.vision.util.ScreenOrientation;

import cpc.robotics.logging.LogServer;
import cpc.robotics.logging.NetLogContext;
import cpc.robotics.vision.BlurExtension;
import cpc.robotics.vision.CameraControlExtension;
import cpc.robotics.vision.CameraStatsExtension;
import cpc.robotics.vision.ImageRotationExtension;
import cpc.robotics.vision.JewelsDetector;
import cpc.robotics.vision.JewelsExtension;
import cpc.robotics.vision.VisionExtension;
import cpc.robotics.vision.VisionHelper;

@Autonomous(name="Test Logger" )
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
