package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public enum RobotType {
    COMPETITION, TEST;

    // Hold the robot type for future runs because it does take some time to
    // detect
    private static RobotType type = null;

    /**
     * SDK Trick: all test bot configs have an Analog Input device called "SW" (software)
     * hooked up to nothing. The test bot will raise no error, whereas the
     * competition bot will raise an error.
     */
    public static RobotType detect(HardwareMap map){
        if (type != null)
            return type;

        try {
            map.get(AnalogInput.class, "SW");
            return (type = TEST);
        } catch (IllegalArgumentException e) {
            return (type = COMPETITION);
        }
    }
}
