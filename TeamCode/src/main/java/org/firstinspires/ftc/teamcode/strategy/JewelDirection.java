package org.firstinspires.ftc.teamcode.strategy;

public enum JewelDirection {
    LEFT(-1),
    RIGHT(1),
    UNKNOWN(0);
    int power;

    JewelDirection(int power) {
        this.power = power;
    }

    public JewelDirection cycleDirection() {
        switch (this) {
            case LEFT:
                return RIGHT;
            case RIGHT:
                return UNKNOWN;
            default:
                return LEFT;
        }
    }
}