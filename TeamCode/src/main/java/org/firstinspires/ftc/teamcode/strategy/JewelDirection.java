package org.firstinspires.ftc.teamcode.strategy;

import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.hardware.AdafruitGraphix;

public enum JewelDirection {
    LEFT(-1) {
        @Override
        public void displayStatus(Tilerunner tilerunner) {
            try (AdafruitGraphix.Draw ignored = tilerunner.graphix.begin(false)) {
                tilerunner.graphix.fillRect(0, 2, 4, 4, AdafruitGraphix.GREEN);
            }
        }
    },
    RIGHT(1) {
        @Override
        public void displayStatus(Tilerunner tilerunner) {
            try (AdafruitGraphix.Draw ignored = tilerunner.graphix.begin(false)) {
                tilerunner.graphix.fillRect(4, 2, 4, 4, AdafruitGraphix.GREEN);
            }

        }
    },
    UNKNOWN(0) {
        @Override
        public void displayStatus(Tilerunner tilerunner) {
            tilerunner.displayUnknown();
        }
    };
    int power;

    JewelDirection(int power) {
        this.power = power;
    }

    public abstract void displayStatus(Tilerunner tilerunner);

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