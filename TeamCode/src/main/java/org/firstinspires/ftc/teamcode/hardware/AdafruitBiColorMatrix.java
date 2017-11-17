package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import java.util.Arrays;

@SuppressWarnings("WeakerAccess")
@I2cSensor(name = "Adafruit BiColor Matrix", description = "8x8 LED Matrix", xmlTag = "AdafruitBiColorMatrix")
public class AdafruitBiColorMatrix extends I2cDeviceSynchDevice<I2cDeviceSynch> implements I2cAddrConfig {

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x70);
    private final static int WIDTH = 8;
    private final static int HEIGHT = 8;

    public enum Register {
        DISPLAY(0x00),
        SYS_SETUP(0x20),
        DISP_SETUP(0x80),
        ROW_INT_LEVEL(0xA0),
        DIMMING(0xE0),
        ;
        public byte bVal;
        Register(int bVal) { this.bVal = (byte)bVal; }
    }

    public enum DisplayMode {
        DISABLED(0x00),
        SOLID(0x01),
        BLINK_2HZ(0x03),
        BLINK_1HZ(0x05),
        BLINK_SLOW(0x07),
        ;
        public byte bVal;
        DisplayMode(int bVal) { this.bVal = (byte)bVal; }
    }

    public enum Oscillator {
        OFF(0x00),
        ON(0x01),
        ;
        public byte bVal;
        Oscillator(int bVal) { this.bVal = (byte)bVal; }
    }

    public final byte [] buffer = new byte[WIDTH*HEIGHT*2/8];

    private static boolean hasRed(short color) {
        return (color&AdafruitGraphix.RED) != 0;
    }

    private static boolean hasGreen(short color) {
        return (color&AdafruitGraphix.GREEN) != 0;
    }

    public class Graphix extends AdafruitGraphix {

        protected Graphix() {
            super(WIDTH, HEIGHT);
        }

        @Override
        public void drawPixel(int x, int y, short color) {
            if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
                return;
            }
            int pos = y*2;
            if (hasGreen(color)) {
                buffer[pos] |= 0x01;
            } else {
                buffer[pos] &= ~0x01;
            }
            pos++;
            if (hasRed(color)) {
                buffer[pos] |= 0x01;
            } else {
                buffer[pos] &= ~0x01;
            }
        }

        @Override
        public void display() {
            AdafruitBiColorMatrix.this.display();
        }
    };

    private final Graphix graphix = new Graphix();

    public final Graphix getGraphix() {
        return graphix;
    }

    //
    // This uses the HT16K33, with ROW0..ROW7 driving green, and ROW8..ROW15 driving red
    // red+green = 3rd color, yellow
    // Display buffer is divided into 8 column blocks of 16 bit rows occupying addresses 00 through 0F
    //

    public AdafruitBiColorMatrix(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        setBrightness((byte)7);
        setOscillator(Oscillator.ON);
        setDisplayMode(DisplayMode.SOLID);
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
        return "BiColor LED Matrix";
    }

    //
    // Device communication
    //

    private static final byte [] NO_DATA = {};

    public void writeSpecial(Register reg, byte value) {
        this.deviceClient.write((byte)(reg.bVal|(value & 0x0F)), NO_DATA, I2cWaitControl.ATOMIC);
    }

    public void writeStream(Register reg, byte [] data) {
        this.deviceClient.write(reg.bVal, data, I2cWaitControl.ATOMIC);
    }

    public int readUnsignedByte(Register reg) {
        return TypeConversion.unsignedByteToInt(this.deviceClient.read8(reg.bVal));
    }

    public void display() {

        writeStream(Register.DISPLAY, buffer);
    }

    public void setOscillator(Oscillator mode) {
        writeSpecial(Register.SYS_SETUP, mode.bVal);
    }

    public void setDisplayMode(DisplayMode mode) {
        writeSpecial(Register.DISP_SETUP, mode.bVal);
    }

    public void setBrightness(byte b) {
        if (b > 15) {
            b = 15;
        }
        byte [] empty = {};
        writeSpecial(Register.DIMMING, b);
    }
}
