package org.firstinspires.ftc.teamcode.strategy;

import org.firstinspires.ftc.teamcode.EyesightUtil;

/**
 * Finds what Cryptobox Key to go to
 */
@SuppressWarnings("WeakerAccess")
public class PictographStrategy {

    private CryptoboxColumn column = CryptoboxColumn.UNKNOWN;

    public CryptoboxColumn readCryptoboxKey() {
        return column;
    }
}
