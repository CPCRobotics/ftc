package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

import java.util.Collection;
import java.util.Collections;

/**
 * Enables mass changing a group of DcMotors.
 * Getters throw an exception
 */

public class DCMotorGroup implements DcMotor {
    private Collection<DcMotor> dcMotors;

    public DCMotorGroup(Collection<DcMotor> dcMotors) {
        this.dcMotors = dcMotors;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.setMotorType(motorType);
    }

    private DcMotor getSingleMotor() {
        return dcMotors.iterator().next();
    }

    @Override
    public DcMotorController getController() {
        return getSingleMotor().getController();
    }

    @Override
    public int getPortNumber() {
        return getSingleMotor().getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return getSingleMotor().getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return getSingleMotor().getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return getSingleMotor().getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return getSingleMotor().isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return getSingleMotor().getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return getSingleMotor().getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return getSingleMotor().getDirection();
    }

    @Override
    public void setPower(double power) {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.setPower(power);
    }

    @Override
    public double getPower() {
        return getSingleMotor().getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return getSingleMotor().getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return getSingleMotor().getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return getSingleMotor().getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return getSingleMotor().getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.close();
    }
}
