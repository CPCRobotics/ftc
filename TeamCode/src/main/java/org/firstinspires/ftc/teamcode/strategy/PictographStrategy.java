package org.firstinspires.ftc.teamcode.strategy;

import org.firstinspires.ftc.teamcode.EyesightUtil;

/**
 * Finds what Cryptobox Key to go to
 */
@SuppressWarnings("WeakerAccess")
public class PictographStrategy {

    private static CryptoboxColumn column = CryptoboxColumn.UNKNOWN;

    public static CryptoboxColumn readCryptoboxKey() {
        column = CryptoboxColumn.fromVuMark(EyesightUtil.getPictograph());
        return getCryptoboxKey();
    }

    public static CryptoboxColumn getCryptoboxKey() {
        return column;
    }
}
