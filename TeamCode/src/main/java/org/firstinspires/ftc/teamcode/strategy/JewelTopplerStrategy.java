package org.firstinspires.ftc.teamcode.strategy;

import android.content.Context;
import android.hardware.Camera;

import org.firstinspires.ftc.teamcode.BusyWaitHandler;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;
import org.lasarobotics.vision.util.ScreenOrientation;

import cpc.robotics.vision.BlurExtension;
import cpc.robotics.vision.CameraControlExtension;
import cpc.robotics.vision.CameraStatsExtension;
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
    private final JewelsExtension jewels = new JewelsExtension();
    private final ImageRotationExtension imageRot = new ImageRotationExtension();
    private final CameraControlExtension cameraControl = new CameraControlExtension();
    private final CameraStatsExtension stats = new CameraStatsExtension();

    public JewelTopplerStrategy(TeamPosition position, BusyWaitHandler waitHandler, Context context,
                                Tilerunner tilerunner) {
        this.position = position;
        this.waitHandler = waitHandler;
        this.tilerunner = tilerunner;

        visionHelper = new VisionHelper(context, Camera.CameraInfo.CAMERA_FACING_BACK, 500, 500);
        visionHelper.addExtensions(blur, jewels, imageRot, cameraControl, stats);
        blur.setBlurWidth(5);
        imageRot.disableAutoRotate();
        imageRot.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE);
        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();
    }

    enum JewelDirection {
        LEFT(-1, "LEFT"), RIGHT(1, "RIGHT");
        int power;
        String value;

        JewelDirection(int power, String value) {
            this.power = power;
            this.value = value;
        }

        public String getString() { return value; }
    }

    public void toppleEnemyJewel() throws InterruptedException {
        visionHelper.enable();

        JewelDirection jd = locateEnemyJewel();
        Twigger.getInstance()
                .sendOnce("Enemy jewel detected: " + jd.toString());
        takeDownEnemyJewel(jd);

        // Turn off camera to let Vuphoria work in Pictograph Strategy
        visionHelper.disable();
    }

    private JewelDirection locateEnemyJewel() throws InterruptedException {
        while (waitHandler.isActive()) {
            JewelsDetector.JewelAnalysis analysis = jewels.getAnalysis();
            // Confidence level of < 25% is not reliable; wait until it is reliable
            //      (the jewels extension might have to think more)
            if (analysis.getConfidence() < .25)
                continue;

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

    private void takeDownEnemyJewel(JewelDirection jd) {
        tilerunner.activateJewelWhacker(waitHandler);

        double direction;
        switch (jd) {
            case LEFT:
                direction = 1;
                break;
            case RIGHT:
            default:
                direction = -1;
                break;
        }

        tilerunner.move(waitHandler, 1, 2*direction);
        tilerunner.retractJewelWhacker(waitHandler);
        tilerunner.move(waitHandler, 1, -2*direction);
    }
}
