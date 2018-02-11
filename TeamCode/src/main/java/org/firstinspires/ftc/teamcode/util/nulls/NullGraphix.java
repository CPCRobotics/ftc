package org.firstinspires.ftc.teamcode.util.nulls;

import org.firstinspires.ftc.teamcode.hardware.AdafruitGraphix;

public class NullGraphix extends AdafruitGraphix {
    /**
     * Initialize null display
     */
    public NullGraphix() {
        super(0, 0);
    }

    @Override
    public void drawPixel(int x, int y, short color) {
        // does nothing
    }

    @Override
    public void display() {
        // does nothing
    }
}
