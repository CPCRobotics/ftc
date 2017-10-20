/*
 * Red/Blue Jewel detection code testable vision op mode.
 *
 * Based on LASA Robotics Beacon detection code.
 * Original source Copyright (c) 2016 Arthur Pachachura, LASA Robotics, and contributors
 * MIT licensed
 */

package cpcs.vision;

/**
 * Vision Op Mode designed ONLY for testing applications, such as the Camera Test Activity
 * This OpMode essentially unifies testing applications and the robot controller
 */
public abstract class JewelsTestableVisionOpMode extends JewelsVisionOpMode {

    /**
     * Creates the Testable OpMode.
     */
    public JewelsTestableVisionOpMode() {
        super(false); //disable OpenCV core functions
    }

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();
    }

    @Override
    public void stop() {
        super.stop();
    }
}