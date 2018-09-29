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
        if (servos.size() > 0)
            return servos.iterator().next().getController();
        return null;
    }

    @Override
    public int getPortNumber() {
        if (servos.size() > 0)
            return servos.iterator().next().getPortNumber();
        return 0;
    }

    @Override
    public void setDirection(Direction direction) {
        for (Servo s : servos)
            s.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        if (servos.size() > 0)
            return servos.iterator().next().getDirection();
        return Direction.FORWARD;
    }

    @Override
    public void setPosition(double position) {
        for (Servo s : servos)
            s.setPosition(position);
    }

    @Override
    public double getPosition() {
        if (servos.size() > 0)
            return servos.iterator().next().getPosition();
        return 0;
    }

    @Override
    public void scaleRange(double min, double max) {
        for (Servo s : servos)
            s.scaleRange(min, max);
    }

    @Override
    public Manufacturer getManufacturer() {
        if (servos.size() > 0)
            return servos.iterator().next().getManufacturer();
        return null;
    }

    @Override
    public String getDeviceName() {
        if (servos.size() > 0)
            return servos.iterator().next().getDeviceName();
        return "EMPTY ServoGroup";
    }

    @Override
    public String getConnectionInfo() {
        if (servos.size() > 0)
            return servos.iterator().next().getConnectionInfo();
        return "EMPTY ServoGroup";
    }

    @Override
    public int getVersion() {
        if (servos.size() > 0)
            return servos.iterator().next().getVersion();
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
