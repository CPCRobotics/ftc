package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

import java.util.Collection;

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

    @Override
    public DcMotorController getController() {
        throw new UnsupportedOperationException();
    }

    @Override
    public int getPortNumber() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setPowerFloat() {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setTargetPosition(int position) {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean isBusy() {
        throw new UnsupportedOperationException();
    }

    @Override
    public int getCurrentPosition() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setMode(RunMode mode) {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setDirection(Direction direction) {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setPower(double power) {
        for (DcMotor dcMotor : dcMotors)
            dcMotor.setPower(power);
    }

    @Override
    public double getPower() {
        throw new UnsupportedOperationException();
    }

    @Override
    public Manufacturer getManufacturer() {
        throw new UnsupportedOperationException();
    }

    @Override
    public String getDeviceName() {
        throw new UnsupportedOperationException();
    }

    @Override
    public String getConnectionInfo() {
        throw new UnsupportedOperationException();
    }

    @Override
    public int getVersion() {
        throw new UnsupportedOperationException();
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
