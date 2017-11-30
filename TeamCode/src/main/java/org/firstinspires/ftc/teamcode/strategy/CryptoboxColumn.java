package org.firstinspires.ftc.teamcode.strategy;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.hardware.AdafruitGraphix;

/**
 * The three different cryptobox columns.
 * <p>
 * Enum used for decision making
 */
enum CryptoboxColumn {
    LEFT {
        @Override
        public void displayPosition(Tilerunner tilerunner) {
            AdafruitGraphix g = tilerunner.graphix;
            try (AdafruitGraphix.Draw ignored = g.begin(true)) {
                g.fillRect(0, 0, 2, 8, AdafruitGraphix.YELLOW);
            }
        }
    },
    CENTER {
        @Override
        public void displayPosition(Tilerunner tilerunner) {
            AdafruitGraphix g = tilerunner.graphix;
            try (AdafruitGraphix.Draw ignored = g.begin(true)) {
                g.fillRect(3, 0, 2, 8, AdafruitGraphix.YELLOW);
            }
        }
    },
    RIGHT {
        @Override
        public void displayPosition(Tilerunner tilerunner) {
            AdafruitGraphix g = tilerunner.graphix;
            try (AdafruitGraphix.Draw ignored = g.begin(true)) {
                g.fillRect(6, 0, 2, 8, AdafruitGraphix.YELLOW);
            }
        }
    },
    UNKNOWN {
        @Override
        public void displayPosition(Tilerunner tilerunner) {
            tilerunner.displayUnknown();
        }
    };

    public static CryptoboxColumn fromVuMark(RelicRecoveryVuMark vuMark) {
        switch (vuMark) {
            case LEFT:
                return LEFT;
            case RIGHT:
                return RIGHT;
            case CENTER:
                return CENTER;
            default:
                return UNKNOWN;
        }
    }

    public abstract void displayPosition(Tilerunner tilerunner);
}
