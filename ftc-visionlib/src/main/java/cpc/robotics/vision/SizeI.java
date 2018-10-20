package cpc.robotics.vision;

public class SizeI {
    public final int width;
    public final int height;

    public SizeI(int width, int height) {
        this.width = width;
        this.height = height;
    }

    public String toString() {
        return "Width: " + String.valueOf(width) + " Height: " + String.valueOf(height);
    }
}
