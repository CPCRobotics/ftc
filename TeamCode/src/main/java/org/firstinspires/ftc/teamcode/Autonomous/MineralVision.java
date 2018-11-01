package org.firstinspires.ftc.teamcode.Autonomous;

/**
 * Finds the the jewel to topple and topples it
 */
public class MineralVision {
/*
    private final TeamPosition position;
    private final VisionHelper visionHelper;
    private final TileRunner tilerunner;

    private final JewelsExtension jewels = new JewelsExtension();

    public MineralVision(TeamPosition position, Context context, TileRunner tilerunner)
    {
        this.position = position;
        this.tilerunner = tilerunner;

        visionHelper = new VisionHelper(context, Camera.CameraInfo.CAMERA_FACING_BACK, 900, 900);
        BlurExtension blur = new BlurExtension();
        CropExtension crop = new CropExtension();
        ImageRotationExtension imageRot = new ImageRotationExtension();
        CameraControlExtension cameraControl = new CameraControlExtension();
        CameraStatsExtension cameraStats = new CameraStatsExtension();
        visionHelper.addExtensions(crop, blur, jewels, imageRot, cameraControl, cameraStats);

        crop.setBounds(-10.0, 0.0, 50.0, 50.0);
        blur.setBlurWidth(5);
        imageRot.disableAutoRotate();
        imageRot.setIsUsingSecondaryCamera(false);
        imageRot.setZeroOrientation(ScreenOrientation.LANDSCAPE_REVERSE);
        imageRot.setActivityOrientationFixed(ScreenOrientation.LANDSCAPE_REVERSE);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        jewels.enableDebug();
        */
    }

    /**
     * @return offset
     */

    /*
    public double toppleEnemyJewel() throws InterruptedException {
        JewelDirection jd;

        try {
            visionHelper.enable();
            jd = locateEnemyJewel();
        } catch (Exception e) {
            jd = JewelDirection.UNKNOWN;
        } finally {
            visionHelper.disable();
        }

        boolean jewelDetected = (jd != JewelDirection.UNKNOWN);
        tilerunner.setGreen(jewelDetected);


        double offset = 0;
        if (jd == JewelDirection.UNKNOWN) {
            Twigger.getInstance()
                    .sendOnce("Can't locate enemy jewel");
        } else {
            Twigger.getInstance()
                    .sendOnce("Enemy jewel detected: " + jd.name());
            offset = takeDownEnemyJewel(jd);
        }

        // Turn off camera to let Vuphoria work in Pictograph Strategy
        visionHelper.disable();

        Twigger.getInstance()
                .sendOnce("Finished Jewel Toppling");
        return offset;
    }

    private JewelDirection locateEnemyJewel() throws InterruptedException {
        ElapsedTime time = new ElapsedTime();

        while (waitHandler.isActive()) {


            // Waits for a full half second or until confidence is high enough
            if ((analysis = jewels.getBestAnalysis()).get            JewelsDetector.JewelAnalysis analysis;
            Confidence() < 0.75 && time.seconds() < 0.5) {
                Twigger.getInstance()
                        .sendOnce("Confidence not high enough: " + analysis.getConfidence());

                Thread.sleep(20);
                continue;
            }

            Twigger.getInstance()
                    .sendOnce("Analysis stopped at confidence level " + analysis.getConfidence());

            // Bad Analysis
            if (analysis.getConfidence() < 0.75) {
                Twigger.getInstance()
                        .sendOnce("WARN: Confidence unreliable");
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

    private double takeDownEnemyJewel(@NonNull JewelDirection jd) throws InterruptedException {
        tilerunner.activateJewelWhacker(waitHandler);
        final double TOPPLE_DISTANCE = 3;

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
        tilerunner.moveInches(waitHandler, 1, TOPPLE_DISTANCE*direction);
        Thread.sleep(100);
        Thread.sleep(100);
        tilerunner.retractJewelWhacker();
        Thread.sleep(100);

        return direction*TOPPLE_DISTANCE;
    }

    */

//}
