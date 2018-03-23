package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

/**
 * This is the AdaFruit 8x8 bi-color matrix able to display three colors - reg, green and orange.
 * Code here is heavily influenced by AdaFruit_LEDBackpack.h distributed under MIT license.
 * The I2C driver chip used by the display is the HT16K33.
 */
@I2cSensor(name = "Adafruit BiColor Matrix", description = "8x8 LED Matrix", xmlTag = "AdafruitBiColorMatrix")
public class AdafruitBiColorMatrix extends I2cDeviceSynchDevice<I2cDeviceSynch> implements I2cAddrConfig {

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x70);
    private final static int WIDTH = 8;
    private final static int HEIGHT = 8;

    /**
     * Pseudo-registers. The range 00-1F behaves like registers. Everything else uses lower 4
     * bits as data, which doesn't work so well with the Robot controller. However the interaction
     * between ModernRobotics controller and HT16K33 appears to be tolerant to sending a dummy
     * data byte.
     */
    public enum Register {
        DISPLAY(0x00),
        SYS_SETUP(0x20),
        DISP_SETUP(0x80),
        DIMMING(0xE0),
        ;
        public final byte bVal;
        Register(int bVal) { this.bVal = (byte)bVal; }
    }

    public enum DisplayMode {
        DISABLED(0x00),
        SOLID(0x01),
        BLINK_2HZ(0x03),
        BLINK_1HZ(0x05),
        BLINK_SLOW(0x07),
        ;
        public final byte bVal;
        DisplayMode(int bVal) { this.bVal = (byte)bVal; }
    }

    public enum Oscillator {
        OFF(0x00),
        ON(0x01),
        ;
        public final byte bVal;
        Oscillator(int bVal) { this.bVal = (byte)bVal; }
    }

    public final byte [] buffer = new byte[WIDTH*HEIGHT*2/8];
    private int rotation = 0;

    /**
     * Helper - if any red appears in the selected color, light the RED LED
     * @param color 16-bit color
     * @return true if RED LED should be lit.
     */
    private static boolean hasRed(short color) {
        return (color&AdafruitGraphix.RED) != 0;
    }

    /**
     * Helper - if any green appears in the selected color, light the GREEN LED
     * @param color 16-bit color
     * @return true if GREEN led should be lit.
     */
    private static boolean hasGreen(short color) {
        return (color&AdafruitGraphix.GREEN) != 0;
    }

    /**
     * Implement minimum functionality for the Graphix display class to enable the display.
     */
    public class Graphix extends AdafruitGraphix {

        protected Graphix() {
            super(WIDTH, HEIGHT);
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void drawPixel(int x, int y, short color) {
            int t;
            switch (rotation) {
                case 1:
                  t = x;
                  x = y;
                  y = t;
                  x = 8 - x - 1;
                  break;

                case 2:
                    x = 8 - x - 1;
                    y = 8 - y - 1;
                    break;

                case 3:
                    t = x;
                    x = y;
                    y = t;
                    y = 8 - y - 1;
                    break;
            }
            if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
                return;
            }
            int pos = y*2;
            byte mask = (byte)(1<<x);
            if (hasGreen(color)) {
                buffer[pos] |= mask;
            } else {
                buffer[pos] &= ~mask;
            }
            pos++;
            if (hasRed(color)) {
                buffer[pos] |= mask;
            } else {
                buffer[pos] &= ~mask;
            }
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void display() {
            AdafruitBiColorMatrix.this.display();
        }

        /**
         * {@inheritDoc}
         */
        @Override
        public void fillScreen(short color) {
            // optimizes clearScreen() and fillScreen()
            byte setRed = hasRed(color) ? (byte)0xff : 0;
            byte setGreen = hasGreen(color) ? (byte)0xff : 0;
            for(int i = 0; i < buffer.length; i+=2) {
                buffer[i]   = setGreen;
                buffer[i+1] = setRed;
            }
        }
    }

    private final Graphix graphix = new Graphix();

    public final Graphix getGraphix() {
        return graphix;
    }
    public void setRotation(int rotation) {
        this.rotation = rotation & 3;
    }

    //
    // This uses the HT16K33, with ROW0..ROW7 driving green, and ROW8..ROW15 driving red
    // red+green = 3rd color, yellow
    // Display buffer is divided into 8 column blocks of 16 bit rows occupying addresses 00 through 0F
    //

    /**
     * {@inheritDoc}
     */
    public AdafruitBiColorMatrix(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        this.deviceClient.engage();
    }

    /**
     * {@inheritDoc}
     * @return
     */
    @Override
    protected boolean doInitialize() {
        // Called when op-mode changes
        this.deviceClient.setLoggingTag("Matrix");
        this.deviceClient.enableWriteCoalescing(false);
        graphix.initialize();
        // Repeat this twice in case bus is in a bad state for any reason
        setDisplayMode(DisplayMode.DISABLED);
        setDisplayMode(DisplayMode.DISABLED);
        // This is sufficient brightness and avoids overloading the controller
        setBrightness((byte)2);
        // effectively clear screen prior to showing display
        display();
        // Show display by enabling oscillator and enabling the display itself (no blinking)
        setOscillator(Oscillator.ON);
        setDisplayMode(DisplayMode.SOLID);
        setBrightness((byte)2);
        return true;
    }

    //----------------------------------------------------------------------------------------------
    // I2cAddressConfig
    //----------------------------------------------------------------------------------------------

    /**
     * {@inheritDoc}
     */
    @Override public void setI2cAddress(I2cAddr newAddress) {
        this.deviceClient.setI2cAddress(newAddress);
    }

    /**
     * {@inheritDoc}
     */
    @Override public I2cAddr getI2cAddress() {
        return this.deviceClient.getI2cAddress();
    }

    //----------------------------------------------------------------------------------------------
    // HardwareDevice
    //----------------------------------------------------------------------------------------------

    /**
     * {@inheritDoc}
     */
    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String getDeviceName() {
        return "Adafruit BiColor LED Matrix";
    }

    //
    // Device communication
    //

    private static final byte [] DUMMY_DATA = { 0 };

    /**
     * Workaround the I2C behavior of the actual device. Combine upper 4 bits of
     * the "register", and lower 4 bits of "value". This is the register address.
     * Write the value 0 to this register address.
     * @param reg Selected register
     * @param value 4 bit control value
     */
    public void writeSpecial(Register reg, byte value) {
        // Make sure all writes go through, order is important
        this.deviceClient.write((byte)(reg.bVal|(value & 0x0F)), DUMMY_DATA, I2cWaitControl.ATOMIC);
    }

    /**
     * Follow normal register model, write data to the given register. As this is register mapped,
     * and order is not important, writes can be coalesced.
     * @param reg Base register
     * @param data Array of data to write
     */
    public void writeStream(Register reg, byte [] data) {
        this.deviceClient.write(reg.bVal, data, I2cWaitControl.NONE);
    }

    /**
     * Refresh display. This is an alias and implementation of updating the display
     */
    public void display() {

        writeStream(Register.DISPLAY, buffer);
    }

    /**
     * Set oscillator mode.
     * @param mode OFF/ON
     */
    public void setOscillator(Oscillator mode) {
        writeSpecial(Register.SYS_SETUP, mode.bVal);
    }

    /**
     * Set display mode.
     * @param mode DISABLED/SOLID/BLINK
     */
    public void setDisplayMode(DisplayMode mode) {
        writeSpecial(Register.DISP_SETUP, mode.bVal);
    }

    /**
     * Set brightness. Be cautious of power used. A solid orange line of display at full brightness
     * will draw about 170mA against a budget of 150mA. A capacitor on the device helps average
     * out power draw, but only on a per-line level.
     * @param brightness 0-15
     */
    public void setBrightness(byte brightness) {
        if (brightness > 15) {
            brightness = 15;
        }
        byte [] empty = {};
        writeSpecial(Register.DIMMING, brightness);
    }
}
