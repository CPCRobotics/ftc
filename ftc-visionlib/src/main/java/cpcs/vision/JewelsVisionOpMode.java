/*
 * Red/Blue Jewel detection code vision op mode.
 *
 * Based on LASA Robotics Beacon detection code.
 * Original source Copyright (c) 2016 Arthur Pachachura, LASA Robotics, and contributors
 * MIT licensed
 */
package cpcs.vision;

import org.lasarobotics.vision.opmode.VisionOpModeCore;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.opmode.extensions.ImageRotationExtension;
import org.lasarobotics.vision.opmode.extensions.VisionExtension;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * Easy-to-use, extensible vision op mode
 * For more custom implementations, use ManualVisionOpMode or modify core extensions in opmode.extensions.*
 */
public abstract class JewelsVisionOpMode extends VisionOpModeCore {

    /***
     * CUSTOM EXTENSION INITIALIZATION
     * <p/>
     * Add your extension here and in the Extensions class below!
     */
    public static final BlurExtension blur = new BlurExtension();
    public static final JewelsExtension jewels = new JewelsExtension();
    public static final ImageRotationExtension rotation = new ImageRotationExtension();
    public static final CameraControlExtension cameraControl = new CameraControlExtension();

    private boolean enableOpenCV = true;
    /**
     * END OF CUSTOM EXTENSION INITIALIZATION
     */

    private int extensions = 0;
    private boolean extensionsInitialized = false;

    public JewelsVisionOpMode() {
        super();
    }

    JewelsVisionOpMode(boolean enableOpenCV) {
        super();
        this.enableOpenCV = enableOpenCV;
    }

    private boolean isEnabled(Extensions extension) {
        return (extensions & extension.id) > 0;
    }

    /**
     * Enable a particular Vision Extension.
     *
     * @param extension Extension ID
     */
    protected void enableExtension(Extensions extension) {
        //Don't initialize extension if we haven't ever called init() yet
        if (extensionsInitialized)
            extension.instance.init(this);

        extensions = extensions | extension.id;
    }

    /**
     * Disable a particular Vision Extension
     *
     * @param extension Extension ID
     */
    private void disableExtension(Extensions extension) {
        extensions -= extensions & extension.id;

        extension.instance.stop(this);
    }

    @Override
    public void init() {
        if (enableOpenCV) super.init();

        for (Extensions extension : Extensions.values())
            if (isEnabled(extension))
                extension.instance.init(this);

        extensionsInitialized = true;
    }

    @Override
    public void loop() {
        if (enableOpenCV) super.loop();

        for (Extensions extension : Extensions.values())
            if (isEnabled(extension))
                extension.instance.loop(this);
    }

    @Override
    public Mat frame(Mat rgba, Mat gray) {

        for (Extensions extension : Extensions.values())
            if (isEnabled(extension)) {
                //Pipe the rgba of the previous point into the gray of the next
//                Imgproc.cvtColor(rgba, gray, Imgproc.COLOR_RGBA2GRAY);
                rgba = extension.instance.frame(this, rgba, null);
            }

        return rgba;
    }

    @Override
    public void stop() {
        super.stop();

        for (Extensions extension : Extensions.values())
            if (isEnabled(extension))
                disableExtension(extension); //disable and stop
    }

    /**
     * List of Vision Extensions
     */
    public enum Extensions {
        BLUR(1, blur),
        JEWELS(2, jewels),
        CAMERA_CONTROL(4, cameraControl),
        ROTATION(8, rotation);

        final int id;
        final VisionExtension instance;

        Extensions(int id, VisionExtension instance) {
            this.id = id;
            this.instance = instance;
        }
    }
}
