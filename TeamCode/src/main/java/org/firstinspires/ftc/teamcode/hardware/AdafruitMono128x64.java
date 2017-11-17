package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

import java.util.Arrays;

@SuppressWarnings("WeakerAccess")
@I2cSensor(name = "Adafruit Mono 128x64", description = "Monochrome OLED 128x64", xmlTag = "AdafruitMono128x64")
public class AdafruitMono128x64 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements I2cAddrConfig {

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x3D);
    private final static int WIDTH = 128;
    private final static int HEIGHT = 64;

    public static interface ControlPair {
        byte getControl();
        byte getData();
    }

    public static class Control implements ControlPair {
        byte ctrl;
        byte bVal;

        public static Control of(int b) {
            return new Control((byte)b);
        }

        public static ControlPair of(ControlPair sup, int b) {
            return new Control(sup.getControl(), (byte)(sup.getData()+b));
        }

        public Control(byte bVal) {
            this((byte)0, bVal);
        }

        public Control(byte ctrl, byte bVal) {
            this.ctrl = ctrl;
            this.bVal = bVal;
        }

        @Override
        public byte getControl() {
            return ctrl;
        }

        @Override
        public byte getData() {
            return bVal;
        }
    }

    public enum ControlOf implements ControlPair {
        SET_LOW_COL(0x00),
        SET_HIGH_COL(0x10),
        MEMORY_MODE(0x20),
        COLUMN_ADDR(0x21),
        PAGE_ADDR(0x22),
        RIGHT_HORIZONTAL_SCROLL(0x26),
        LEFT_HORIZONTAL_SCROLL(0x27),
        VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL(0x29),
        VERTICAL_AND_LEFT_HORIZONTAL_SCROLL(0x2A),
        DEACTIVATE_SCROLL(0x2E),
        ACTIVATE_SCROLL(0x2F),
        SET_START_LINE(0x40),
        SET_CONTRAST(0x81),
        CHARGE_PUMP(0x8D),
        SEG_REMAP(0xA0),
        SET_VERTICAL_SCROLL_AREA(0xA3),
        DISPLAY_ALL_ON_RESUME(0xA4),
        DISPLAY_ALL_ON(0xA5),
        NORMAL_DISPLAY(0xA6),
        INVERT_DISPLAY(0xA7),
        SET_MULTIPLEX(0xA8),
        DISPLAY_OFF(0xAE),
        DISPLAY_ON(0xAF),
        COM_SCAN_INC(0xC0),
        COM_SCAN_DEC(0xC8),
        SET_DISPLAY_OFFSET(0xD3),
        SET_DISPLAY_CLOCK_DIV(0xD5),
        SET_PRECHARGE(0xD9),
        SET_COMPINS(0xDA),
        SET_VCOM_DETECT(0xDB),
        ;
        public byte bVal;
        ControlOf(int bVal) { this.bVal = (byte)bVal; }

        @Override
        public byte getControl() {
            return 0;
        }

        @Override
        public byte getData() {
            return bVal;
        }
    }

    public final byte [] buffer = new byte[WIDTH*HEIGHT/8];

    private static boolean isWhite(short color) {
        return color != 0;
    }

    public class Graphix extends AdafruitGraphix {

        protected Graphix() {
            super(WIDTH, HEIGHT);
        }

        @Override
        public void drawPixel(int x, int y, short color) {
            if (x < 0 || x >= width || y < 0 || y >= height) {
                return;
            }
            if (isWhite(color)) {
                buffer[x + (y/8)*WIDTH] |= 1 << (y&7);
            } else {
                buffer[x + (y/8)*WIDTH] &= ~(1 << (y&7));
            }
        }

        @Override
        public void display() {
            AdafruitMono128x64.this.display();
        }

        @Override
        public void fillScreen(short color) {
            Arrays.fill(buffer, isWhite(color) ? (byte)0xff : 0);
        }

        @Override
        public void drawFastVLine(int x, int y, int h, short color) {
            // TODO: optimize
            super.drawFastVLine(x,y,h,color);
        }

        @Override
        public void drawFastHLine(int x, int y, int w, short color) {
            // TODO: optimize
            super.drawFastHLine(x,y,w,color);
        }
    };

    private final Graphix graphix = new Graphix();

    public final Graphix getGraphix() {
        return graphix;
    }

    public AdafruitMono128x64(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
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
        return "Monochrome OLED 128x64";
    }

    //
    // Device communication
    //

    private static final byte [] NO_DATA = {};

    public void command(ControlPair first, ControlPair ... others) {
        // This doesn't fit into the normal I2C register model
        // it's actually a sequence of commands.
        // the first byte has 2 flags - b7 = Co, b6=D/C#
        // if first byte is zero, next byte is a command (or command data)
        // I2C sequence of <control> <command> pairs
        byte [] buffer = new byte [others.length*2+1];
        buffer[0] = first.getData();
        for(int i = 0; i < others.length; i++) {
            int pos = i*2+1;
            buffer[pos] = others[i].getControl();
            buffer[pos+1] = others[i].getData();
        }
        this.deviceClient.write(first.getControl(), buffer, I2cWaitControl.ATOMIC);
    }

    public void reset(DigitalChannel resetChannel) throws InterruptedException {
        if (resetChannel != null) {
            resetChannel.setMode(DigitalChannel.Mode.OUTPUT);
            resetChannel.setState(true);
            Thread.sleep(1);
            resetChannel.setState(false);
            Thread.sleep(10);
            resetChannel.setState(true);
        }
        // This logic is lifted from Adafruit SSD1306 driver
        command(ControlOf.DISPLAY_OFF,
                ControlOf.SET_DISPLAY_CLOCK_DIV,
                Control.of(0x80),
                ControlOf.SET_MULTIPLEX,
                Control.of(HEIGHT-1),
                ControlOf.SET_DISPLAY_OFFSET,
                Control.of(0), // no offset
                Control.of(ControlOf.SET_START_LINE, 0),
                ControlOf.CHARGE_PUMP,
                Control.of(0x14), // internal VCC
                ControlOf.MEMORY_MODE,
                Control.of(0));
        command(Control.of(ControlOf.SEG_REMAP, 0x1),
                ControlOf.COM_SCAN_DEC,
                ControlOf.SET_COMPINS,
                Control.of(0x12), // valid for 128x64
                ControlOf.SET_CONTRAST,
                Control.of(0xCF), // internal VCC
                ControlOf.SET_PRECHARGE,
                Control.of(0xF1), // internal VCC
                ControlOf.SET_VCOM_DETECT,
                Control.of(0x40));
        command(ControlOf.DISPLAY_ALL_ON_RESUME,
                ControlOf.NORMAL_DISPLAY,
                ControlOf.DEACTIVATE_SCROLL,
                ControlOf.DISPLAY_ON // turns on panel
                );
    }

    public void display() {
        command(ControlOf.COLUMN_ADDR,
                Control.of(0),
                Control.of(WIDTH-1),
                ControlOf.PAGE_ADDR,
                Control.of(0),
                Control.of(7)); // 64-line
        // write in blocks of 16 bytes
        for(int i = 0; i < buffer.length; i+=16) {
            byte [] block = Arrays.copyOfRange(buffer, i, i+16);
            this.deviceClient.write(0x40, block, I2cWaitControl.ATOMIC);
        }
    }
}
