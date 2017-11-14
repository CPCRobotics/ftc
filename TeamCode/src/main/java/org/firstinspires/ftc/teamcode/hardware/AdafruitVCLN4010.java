package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("WeakerAccess")
@I2cSensor(name = "VCLN4010 Range Sensor", description = "an Adafruit VCLN4010 range sensor", xmlTag = "AdafruitVCLN4010RangeSensor")
public class AdafruitVCLN4010 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements DistanceSensor, OpticalDistanceSensor, I2cAddrConfig {

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create8bit(0x26);

    public enum Register {
        FIRST(0x80),
        COMMAND(0x80),
        PRODUCT_ID(0x81),
        PROXIMITY_RATE(0x82),
        IR_LED_CURRENT(0x83),
        AMBIENT_LIGHT_PARAMETER(0x84),
        AMBIENT_LIGHT_RESULT_HIGH(0x85),
        AMBIENT_LIGHT_RESULT_LOW(0x86),
        PROXIMITY_RESULT_HIGH(0x87),
        PROXIMITY_RESULT_LOW(0x88),
        INTERRUPT_CONTROL(0x89),
        LOW_THRESHOLD_HIGH(0x8A),
        LOW_THRESHOLD_LOW(0x8B),
        HIGH_THRESHOLD_HIGH(0x8C),
        HIGH_THRESHOLD_LOW(0x8D),
        INTERRUPT_STATUS(0x8E),
        PROXIMITY_MODULATING_TIMING(0x8F),
        _AMBIENT_IR_LIGHT_(0x90), // considered internal
        LAST(PROXIMITY_MODULATING_TIMING.bVal), // for read window
        ;
        public byte bVal;
        Register(int bVal) { this.bVal = (byte)bVal; }
    }

    public enum ModFreq {
        FREQ_390K625(0),
        FREQ_781K25(1),
        FREQ_1M5625(2),
        FREQ_3M125(3)
        ;
        public byte bVal;
        ModFreq(int bVal) { this.bVal = (byte)bVal; }
    }

    public enum ProximityRate {
        RATE_2(0),
        RATE_4(1),
        RATE_8(2),
        RATE_16(3),
        RATE_31(4),
        RATE_63(5),
        RATE_125(6),
        RATE_250(7)
        ;
        public byte bVal;
        ProximityRate(int bVal) { this.bVal = (byte)bVal; }
    }

    public volatile int ledCurrent = 200;
    public volatile boolean ledOn = false;

    public AdafruitVCLN4010(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);
        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected void setOptimalReadWindow() {
        // Causes I2C reads to occur in the background
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    public void setLedCurrent(int current) {
        if (current < 0 || current > 200) {
            throw new IllegalArgumentException("Current out of range");
        }
        this.ledCurrent = current;
        if (ledOn) {
            enableLed(true); // refresh current
            writeIrLedCurrent((byte) (current / 10));
        }
    }

    public int getLedCurrent() {
        return this.ledCurrent;
    }

    @Override
    protected synchronized boolean doInitialize() {
        writeProximityModulating(ModFreq.FREQ_390K625);
        writeProximityRate(ProximityRate.RATE_125);
        enableLed(true);
        return true;
    }

    //----------------------------------------------------------------------------------------------
    // I2cAddressConfig
    //----------------------------------------------------------------------------------------------

    @Override public void setI2cAddress(I2cAddr newAddress) {
        this.deviceClient.setI2cAddress(newAddress);
    }

    @Override public I2cAddr getI2cAddress() {
        return this.deviceClient.getI2cAddress();
    }

    //----------------------------------------------------------------------------------------------
    // HardwareDevice
    //----------------------------------------------------------------------------------------------

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "VCLN4010 Range Sensor";
    }

    //----------------------------------------------------------------------------------------------
    // DistanceSensor
    //----------------------------------------------------------------------------------------------

    // Math based on visual graph for VCNL4010 at 200mA
    // for approximately exponential range (6,1000) to (100,44)
    // Quick checks suggest this will work from ~3mm to ~140mm
    private static final double COUNT_A = 10000; // count=1000 @ distance=6mm
    private static final double DISTANCE_A = 6;
    private static final double COUNT_B = 44; // count=44 @ distance=100mm
    private static final double DISTANCE_B = 100;
    private static final double LOG_COUNT_A = Math.log(COUNT_A);
    private static final double LOG_DISTANCE_A = Math.log(DISTANCE_A);
    private static final double LOG_COUNT_B = Math.log(COUNT_B);
    private static final double LOG_DISTANCE_B = Math.log(DISTANCE_B);
    private static final double LOG_RAMP =
            (LOG_DISTANCE_B-LOG_DISTANCE_A) / (LOG_COUNT_A-LOG_COUNT_B);

    @Override
    public double getDistance(DistanceUnit unit) {
        int rawProximity = readProximity();
        if (rawProximity <= 0) {
            return DistanceUnit.infinity;
        }
        double logCount = Math.log((double)rawProximity);
        double logDistance = (LOG_COUNT_A-logCount)*LOG_RAMP+LOG_DISTANCE_A;
        double distance = Math.exp(logDistance);
        return unit.fromMm(distance);
    }

    @Override
    public double getLightDetected() {
        return getRawLightDetected() / getRawLightDetectedMax();
    }

    @Override
    public double getRawLightDetected() {
        return readProximity();
    }

    @Override
    public double getRawLightDetectedMax() {
        return 65535;
    }

    @Override
    public void enableLed(boolean enable) {
        writeIrLedCurrent(enable ? (byte)(ledCurrent/10) : 0);
        ledOn = enable;
    }

    @Override
    public String status() {
        return String.format("%s on %s", getDeviceName(), getConnectionInfo());
    }


    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    public void write8(Register reg, byte value) {
        this.write8(reg, value, I2cWaitControl.NONE);
    }

    public void write8(Register reg, byte value, I2cWaitControl waitControl) {
        this.deviceClient.write8(reg.bVal, value, waitControl);
    }

    public int readUnsignedByte(Register reg) {
        return TypeConversion.unsignedByteToInt(this.deviceClient.read8(reg.bVal));
    }

    public int readUnsignedShort(Register reg) {
        return TypeConversion.unsignedShortToInt(
                TypeConversion.byteArrayToShort(
                        this.deviceClient.read(reg.bVal, 2)));
    }

    public int readCommand() {
        return readUnsignedByte(Register.COMMAND);
    }

    public void writeCommand(byte command) {
        write8(Register.COMMAND, command, I2cWaitControl.WRITTEN);
    }

    public int readProductId() {
        // high 4 bits = Product = 2
        // low 4 bits = revision
        return readUnsignedByte(Register.PRODUCT_ID);
    }

    public int readProximityRate() {
        return readUnsignedByte(Register.PROXIMITY_RATE);
    }

    public void writeProximityRate(ProximityRate rate) {
        write8(Register.PROXIMITY_RATE, rate.bVal);
    }

    public int readIrLedCurrent() {
        return readUnsignedByte(Register.IR_LED_CURRENT);
    }

    public void writeIrLedCurrent(byte parameter) {
        // 0-20 = 0-200mA, default 2, 20mA
        if (parameter > 20) {
            parameter = 20;
        }
        write8(Register.IR_LED_CURRENT, parameter);
    }

    public int readAmbientLightParameter() {
        return readUnsignedByte(Register.AMBIENT_LIGHT_PARAMETER);
    }

    public void writeAmbientLightParameter(byte parameter) {
        write8(Register.AMBIENT_LIGHT_PARAMETER, parameter);
    }

    public int readAmbientLight() {
        return readUnsignedShort(Register.AMBIENT_LIGHT_RESULT_HIGH);
    }

    public int readProximity() {
        return readUnsignedShort(Register.PROXIMITY_RESULT_HIGH);
    }

    public int readProximityModulating() {
        return readUnsignedByte(Register.PROXIMITY_MODULATING_TIMING);
    }

    public void writeProximityModulating(ModFreq freq) {
        write8(Register.PROXIMITY_RATE, freq.bVal);
    }


}
