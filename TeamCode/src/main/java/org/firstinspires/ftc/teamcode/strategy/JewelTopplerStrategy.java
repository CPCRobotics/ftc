package org.firstinspires.ftc.teamcode.strategy;

/**
 * Finds the the jewel to topple and topples it
 */

public class JewelTopplerStrategy {

    private JewelTopplerStrategy() {}

    enum JewelDirection {
        LEFT(-1), RIGHT(1);
        int power;

        JewelDirection(int power) {
            this.power = power;
        }
    }

    public static void toppleEnemyJewel() {
        goToJewels();
        JewelDirection jd = locateEnemyJewel();
        takeDownEnemyJewel(jd);
    }

    private static void goToJewels() {
        throw new UnsupportedOperationException();
    }

    private static JewelDirection locateEnemyJewel() {
        throw new UnsupportedOperationException();
    }

    private static void takeDownEnemyJewel(JewelDirection jd) {
        throw new UnsupportedOperationException();
    }
}
