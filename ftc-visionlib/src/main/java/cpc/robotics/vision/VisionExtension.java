/*
 * Copyright (c) 2016 Arthur Pachachura, LASA Robotics, and contributors
 * MIT licensed
 */
package cpc.robotics.vision;

import org.opencv.core.Mat;

import java.io.Closeable;

/**
 * This is a mechanism for chaining image processing that is done on each frame of video retrieved.
 * Call VisionHelper.addExtension() to add extensions in processing order. Extensions can be
 * endabled or disabled, and are activated only when vision processing needs to be done.
 */
public abstract class VisionExtension implements Closeable {

    private boolean enabled = true;
    private boolean active = false;
    protected VisionHelper vision = null;

    public void setVisionHelper(VisionHelper visionHelper) {
        if (this.vision == visionHelper) {
            return;
        }
        this.close();
        this.vision = visionHelper;
        if (enabled && active && visionHelper != null) {
            onEnabled();
        }
    }

    public final void setActive(boolean active) {
        if (active && !this.active) {
            if (enabled && this.vision != null) {
                onEnabled();
            }
            this.active = true;
        } else if (this.active && !active) {
            this.active = false;
            if (enabled && this.vision != null) {
                onDisabled();
            }
        }
    }

    @Override
    public void close() {
        if (vision != null) {
            if (enabled && active) {
                onDisabled();
            }
            vision = null;
        }
        active = false;
    }

    /**
     * Process frame if enabled
     * @param rgba
     * @return modified rgba
     */
    public final Mat frame(Mat rgba) {
        if (vision != null && enabled && active) {
            return onFrame(rgba);
        } else {
            return rgba;
        }
    }

    /**
     * Override to handle frame
     * @param rgba frame
     * @return modified frame
     */
    protected Mat onFrame(Mat rgba) {
        return rgba;
    }

    /**
     * Override is called once extension has been added to helper.
     */
    protected void onCreate() {
    }

    /**
     * Override to handle disabled->enabled transition.
     */
    protected void onEnabled() {
    }

    /**
     * Override to handle enabled->disabled transition
     */
    protected void onDisabled() {
    }

    /**
     * Change state from enabled to disabled or vice-versa. Callbacks
     * onEnabled/onDisabled are called if state changes.
     * @param newState true if enabled
     */
    public void setEnabled(boolean newState) {
        if (vision == null || !active) {
            enabled = newState;
            return;
        }
        if (enabled && !newState) {
            enabled = false;
            onDisabled();
        } else if (newState && !enabled) {
            onEnabled();
            enabled = true;
        }
    }

    /**
     * Effectively setEnabled(true)
     */
    public final void enable() {
        setEnabled(true);
    }

    /**
     * Effectively setEnabled(false)
     */
    public final void disable() {
        setEnabled(false);
    }

    /**
     * @return current enabled state
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * Allows an extension to resize the view.
     * @param viewSize original size
     * @return modified size or original size
     */
    public SizeI resize(SizeI viewSize) {
        return viewSize;
    }
}
