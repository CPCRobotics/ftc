package org.firstinspires.ftc.teamcode.OpModes.feature;

public abstract class Feature {
    /**
     * Uses the feature
     *
     * @param value value of input
     * @return true if feature is active
     */
    public abstract boolean call(double value);

    public final boolean call(boolean value) {
        return call(value ? 1 : 0);
    }
}
