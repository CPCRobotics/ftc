package org.firstinspires.ftc.teamcode.strategy;

import android.content.Context;
import android.hardware.Camera;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;
import org.lasarobotics.vision.util.ScreenOrientation;

import cpc.robotics.vision.BlurExtension;
import cpc.robotics.vision.CameraControlExtension;
import cpc.robotics.vision.CameraStatsExtension;
import cpc.robotics.vision.CropExtension;
import cpc.robotics.vision.ImageRotationExtension;
import cpc.robotics.vision.JewelsDetector;
import cpc.robotics.vision.JewelsExtension;
import cpc.robotics.vision.VisionHelper;

/**
 * Finds the the jewel to topple and topples it
 */

public class JewelTopplerStrategy {

    private final TeamPosition position;
    private final VisionHelper visionHelper;
    private final BusyWaitHandler waitHandler;
    private final Tilerunner tilerunner;

    private final BlurExtension blur = new BlurExtension();
    private final CropExtension crop = new CropExtension();
    private final JewelsExtension jewels = new JewelsExtension();
    private final ImageRotationExtension imageRot = new ImageRotationExtension();
    private final CameraControlExtension cameraControl = new CameraControlExtension();
    private final CameraStatsExtension cameraStats = new CameraStatsExtension();

    public JewelTopplerStrategy(TeamPosition position, BusyWaitHandler waitHandler,
                                Context context, Tilerunner tilerunner) {
        this.position = position;
        this.waitHandler = waitHandler;
        this.tilerunner = tilerunner;

        visionHelper = new VisionHelper(context, Camera.CameraInfo.CAMERA_FACING_BACK, 900, 900);
        visionHelper.addExtensions(crop, blur, jewels, imageRot, cameraControl, cameraStats);

        crop.setBounds(-10, 0, 50, 50);
        blur.setBlurWidth(5);
        imageRot.disableAutoRotate();
        imageRot.setIsUsingSecondaryCamera(false);
        imageRot.setZeroOrientation(ScreenOrientation.LANDSCAPE_REVERSE);
        imageRot.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE_REVERSE);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        jewels.enableDebug();
    }

    enum JewelDirection {
        LEFT(-1, "LEFT"), RIGHT(1, "RIGHT"), UNKNOWN(0, "UNKNOWN");
        int power;
        String value;

        JewelDirection(int power, String value) {
            this.power = power;
            this.value = value;
        }

        public String getString() { return value; }
    }

    public void toppleEnemyJewel() throws InterruptedException {

        JewelDirection jd = JewelDirection.UNKNOWN;

        try {
            visionHelper.enable();
            jd = locateEnemyJewel();
        } finally {
            visionHelper.disable();
        }

        Twigger.getInstance()
                .sendOnce("Enemy jewel detected: " + jd.toString());
        takeDownEnemyJewel(jd);

        // Turn off camera to let Vuphoria work in Pictograph Strategy
        visionHelper.disable();
    }

    private JewelDirection locateEnemyJewel() throws InterruptedException {
        ElapsedTime time = new ElapsedTime();

        while (waitHandler.isActive()) {

            // Wait 3 seconds for the JewelsExtension to analyze the jewels
            if (time.seconds() < 3) {
                Thread.sleep(20);
                continue;
            }

            final JewelsDetector.JewelAnalysis analysis = jewels.getBestAnalysis();

            // Bad Analysis
            if (analysis.getConfidence() < 0.75) {
                Twigger.getInstance()
                        .sendOnce("ERROR: Can't Find Jewel In Time");
                return JewelDirection.UNKNOWN;
            }

            Twigger.getInstance()
                    .addLine(".locateEnemyJewel()")
                        .addData("colors", analysis.getColorString())
                        .addData("confidence", analysis.getConfidenceString())
                        .addData("center", analysis.getCenterString())
                        .done()
                    .update()
                    .remove(".locateEnemyJewel()"); // TODO Add sendLineOnce implementation


            JewelsDetector.JewelColor leftColor = analysis.getLeftColor();

            // Decide position of enemy color
            if (leftColor == JewelsDetector.JewelColor.RED && position.isRed())
                return JewelDirection.RIGHT;
            else if (leftColor == JewelsDetector.JewelColor.RED && position.isBlue())
                return JewelDirection.LEFT;
            else if (leftColor == JewelsDetector.JewelColor.BLUE && position.isRed())
                return JewelDirection.LEFT;
            else if (leftColor == JewelsDetector.JewelColor.BLUE && position.isBlue())
                return JewelDirection.RIGHT;

        }

        // Waithandler showed opmode is inactive; throw
        //      InterruptedException because it was interrupted
        throw new InterruptedException();
    }

    private void takeDownEnemyJewel(JewelDirection jd) throws InterruptedException {
        tilerunner.activateJewelWhacker(waitHandler);

        double direction;
        switch (jd) {
            case LEFT:
                direction = -1;
                break;
            case RIGHT:
            default:
                direction = 1;
                break;
        }

        tilerunner.longSleep(waitHandler, 250);
        tilerunner.move(waitHandler, 1, 3*direction);
        tilerunner.retractJewelWhacker();
        tilerunner.move(waitHandler, 1, -3*direction);
    }
}
