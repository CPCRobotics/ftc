package org.firstinspires.ftc.teamcode.strategy;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * The three different cryptobox columns.
 * <p>
 * Enum used for decision making
 */
enum CryptoboxColumn {
    LEFT, CENTER, RIGHT, UNKNOWN;

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
}
