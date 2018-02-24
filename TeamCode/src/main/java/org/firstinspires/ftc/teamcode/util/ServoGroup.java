package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import java.util.Arrays;
import java.util.Collection;

/**
 * Lots of servos pretend to be one servo
 */
public class ServoGroup implements Servo {
    private final Collection<? extends Servo> servos;

    public ServoGroup(Collection<? extends Servo> servos) {
        this.servos = servos;
    }

    public ServoGroup(Servo... servos) {
        this(Arrays.asList(servos));
    }

    @Override
    public ServoController getController() {
        for (Servo s : servos)
            return s.getController();
        return null;
    }

    @Override
    public int getPortNumber() {
        for (Servo s : servos)
            return s.getPortNumber();
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {
        for (Servo s : servos)
            s.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        for (Servo s : servos)
            return s.getDirection();
        return Direction.FORWARD;
    }

    @Override
    public void setPosition(double position) {
        for (Servo s : servos)
            s.setPosition(position);
    }

    @Override
    public double getPosition() {
        for (Servo s : servos)
            return s.getPosition();
        return 0;
    }

    @Override
    public void scaleRange(double min, double max) {
        for (Servo s : servos)
            s.scaleRange(min, max);
    }

    @Override
    public Manufacturer getManufacturer() {
        for (Servo s : servos)
            return s.getManufacturer();
        return null;
    }

    @Override
    public String getDeviceName() {
        for (Servo s : servos)
            return s.getDeviceName();
        return "EMPTY ServoGroup";
    }

    @Override
    public String getConnectionInfo() {
        for (Servo s : servos)
            return s.getConnectionInfo();
        return "EMPTY ServoGroup";
    }

    @Override
    public int getVersion() {
        for (Servo s : servos)
            return s.getVersion();
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        for (Servo s : servos)
            s.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        for (Servo s : servos)
            s.close();
    }
}
