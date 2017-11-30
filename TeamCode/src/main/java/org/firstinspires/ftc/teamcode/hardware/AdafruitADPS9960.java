package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import com.qualcomm.hardware.ams.AMSColorSensor;
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

import java.nio.ByteOrder;

@I2cSensor(name = "Adafruit ADPS9960 Range Sensor", description = "an Adafruit ADPS9960 range sensor", xmlTag = "AdafruitADPS9960")
public class AdafruitADPS9960 extends I2cDeviceSynchDevice<I2cDeviceSynch> implements ProximitySensor, I2cAddrConfig {

    public final static String TAG = "ADPS9960";
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x39);
    public final static int DEFAULT_ADC_INTEG_TIME = 100; // ms
    public final static int DEFAULT_WAIT_TIME = 30; // ms - wait between cycles
    public final static LedPPLen DEFAULT_PPLEN = LedPPLen.PPLEN_8us;
    public final static int DEFAULT_PROX_COUNT = 16;
    public final static ProxGain DEFAULT_PROX_GAIN = ProxGain.GAIN_2x;
    public final static AdcGain DEFAULT_ADC_GAIN = AdcGain.GAIN_16x;
    public final static double TIME_MULT = 2.78; // ms
    public final static int DEVICE_ID_1 = 0xAB; // confirms device operation
    public final static int DEVICE_ID_2 = 0x9C; // alternative ID

    public enum Register {
        // RAM is available to use as RAM (why?)
        // However is used as FIFO in gesture mode
        RAM_OR_FIFO(0x00),
        // General registers
        ENABLE(0x80),   // Enable states and interrupts
        ATIME(0x81),    // ADC Integration time (for saturation adjustment)
        WTIME(0x83),    // Wait time (for power management)
        AILTL(0x84),    // ALS interrupts low threshold
        AILTH(0x85),
        AIHTL(0x86),    // ALS interrupts high threshold
        AIHTH(0x87),
        PILT(0x89),     // Proximity interrupts low/high threshold
        PIHT(0x8B),     // also way of identifying near/far reference
        WINDOW_START(0x8C), // interesting register window (have to limit to 26 bytes
        PERS(0x8C),     // Interrupt persistence filters
        CONFIG1(0x8D),  // Mostly reserved, write 0x60 (normal) or 0x62 (long wait)
        PPULSE(0x8E),   // Proximity pulse count and length (effectively sensitivity)
        CONTROL(0x8F),  // Proximity gain control (sensitivity and power management)
        CONFIG2(0x90),  // Interrupts and LED power boost
        ID(0x92),       // Device ID (for health check) = 0xAB
        STATUS(0x93),   // Device Status
        // Ambient light RGBC levels - read as 8 byte block
        CDATAL(0x94),   // "Clear" low/high (ambient light)
        CDATAH(0x95),
        RDATAL(0x96),   // Red low/high
        RDATAH(0x97),
        GDATAL(0x98),   // Green low/high
        GDATAH(0x99),
        BDATAL(0x9A),   // Blue low/high
        BDATAH(0x9B),
        PDATA(0x9C),    // Proximity data (inverse square?)
        POFFSET_UR(0x9D), // Proximity offset - to reduce cross-talk
        POFFSET_DL(0x9E),
        CONFIG3(0x9F),  // disable photodiodes and compensate
        WINDOW_END(0xA0), // end of window range (first non-window byte)
        // gestures
        GPENTH(0xA0),
        GEXTH(0xA1),
        GCONF1(0xA2),
        GCONF2(0xA3),
        GOFFSET_U(0xA4),
        GOFFSET_D(0xA5),
        GPULSE(0xA6),
        GOFFSET_L(0xA7),
        GOFFSET_R(0xA9),
        GCONF3(0xAA),
        GCONF4(0xAB),
        GFLVL(0xAE),
        GSTATUS(0xAF),
        // Interrupts
        IFORCE(0xE4),
        PICLEAR(0xE5),
        CICLEAR(0xE6),
        AICLEAR(0xE7),
        // Read gesture FIFOs
        // expected to read multiple bytes depending on GFLVL
        GFIFO_U(0xFC),
        GFIFO_D(0xFD),
        GFIFO_L(0xFE),
        GFIFO_R(0xFF),
        ;
        public byte bVal;
        Register(int bVal) { this.bVal = (byte)bVal; }
    }

    public enum EnableFlags {
        POWER_ON(0x01),
        ALS(0x02),
        PROXIMITY(0x04),
        WAIT(0x08),
        ALS_INT(0x10),
        PROXIMITY_INT(0x20),
        GESTURES(0x40),
        ;
        public byte bVal;
        EnableFlags(int bVal) { this.bVal = (byte)bVal; }
    }

    public enum LedPPLen {
        // longer the pulse, more sensitive to longer distance
        PPLEN_4us(0),
        PPLEN_8us(1),
        PPLEN_16us(2),
        PPLEN_32us(3)
        ;
        public byte bVal;
        LedPPLen(int bVal) { this.bVal = (byte)bVal; }
    }

    public enum LedLDrive {
        // smaller current, less power and less sensitive to longer distance
        LDRIVE_100mA(0),
        LDRIVE_50mA(1),
        LDRIVE_25mA(2),
        LDRIVE_12mA(3)
        ;
        public byte bVal;
        LedLDrive(int bVal) { this.bVal = (byte)bVal; }
    }

    public enum LedBoost {
        // regain some of the lost LED power (up to 100mA)
        BOOST_100pc(0),
        BOOST_150pc(1),
        BOOST_200pc(2),
        BOOST_300pc(3)
        ;
        public byte bVal;
        LedBoost(int bVal) { this.bVal = (byte)bVal; }
    }

    public enum AdcGain {
        // bigger the gain, more saturated the color
        GAIN_1x(0),
        GAIN_4x(1),
        GAIN_16x(2),
        GAIN_64x(3)
        ;
        public byte bVal;
        AdcGain(int bVal) { this.bVal = (byte)bVal; }
    }

    public enum ProxGain {
        // bigger the gain, more saturated the signal
        GAIN_1x(0),
        GAIN_2x(1),
        GAIN_4x(2),
        GAIN_8x(3)
        ;
        public byte bVal;
        ProxGain(int bVal) { this.bVal = (byte)bVal; }
    }

    public AdafruitADPS9960(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);
        Log.v(TAG, "Created sensor");
        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected void setOptimalReadWindow() {
        // Causes I2C reads to occur in the background
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.WINDOW_START.bVal,
                Register.WINDOW_END.bVal - Register.WINDOW_START.bVal,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    @Override
    protected synchronized boolean doInitialize() {
        Log.v(TAG, "Initializing");
        //this.deviceClient.setLogging(true);
        //this.deviceClient.setLoggingTag(TAG);
        int id = readId();
        if (id != DEVICE_ID_1 && id != DEVICE_ID_2) {
            Log.v(TAG, String.format("Initializing failed, id = %02x", id));
            return false;
        }
        setAdcIntegTime(DEFAULT_ADC_INTEG_TIME);
        setWaitTime(DEFAULT_WAIT_TIME);
        setLedBoost(LedBoost.BOOST_100pc);
        setLedLDrive(LedLDrive.LDRIVE_100mA);
        setAdcGain(DEFAULT_ADC_GAIN);
        setProxGain(DEFAULT_PROX_GAIN);
        setProxPulse(DEFAULT_PPLEN, DEFAULT_PROX_COUNT);
        write8(Register.CONFIG3, (byte)0); // in case someone modified this config
        enableOnly(EnableFlags.PROXIMITY, EnableFlags.ALS, EnableFlags.POWER_ON);
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
        return "Adafruit ADPS9960 Range Sensor";
    }

    //----------------------------------------------------------------------------------------------
    // DistanceSensor
    //----------------------------------------------------------------------------------------------

    @Override
    public double getDistance(DistanceUnit unit) {

        // I = P / D^2
        // 1/I = D^2/P
        // SQRT(P/I) = D where P is a constant
        // Q.SQRT(1/I) = D where Q is SQRT(P)
        //
        // The assumption in this code is that '1' is returned for 100mm, and that distance
        // is linear. This is good enough for practical purposes.
        // Actual value will depend on reflectivity.
        //

        double light_adj = getLightDetected();
        double distance = 100.0 * light_adj; // approximate distance in mm
        return unit.fromMm(distance);
    }

    @Override
    public double getLightDetected() {
        return Math.sqrt(1.0 / Math.max(1, readProximityData()));
    }

    @Override
    public double getRawLightDetected() {
        return readProximityData();
    }

    @Override
    public double getRawLightDetectedMax() {
        return 255;
    }

    @Override
    public void enableLed(boolean enable) {
        if (enable) {
            enable(EnableFlags.PROXIMITY);
        } else {
            disable(EnableFlags.PROXIMITY);
        }
    }

    @Override
    public String status() {
        return String.format("%s on %s", getDeviceName(), getConnectionInfo());
    }


    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    public void write8(Register reg, byte value) {
        this.write8(reg, value, I2cWaitControl.ATOMIC);
    }

    public void write8(Register reg, byte value, I2cWaitControl waitControl) {
        this.deviceClient.write8(reg.bVal, value, waitControl);
    }

    public int readUnsignedByte(Register reg) {
        return TypeConversion.unsignedByteToInt(this.deviceClient.read8(reg.bVal));
    }

    public int readUnsignedShort(Register reg) {
        // Device is little-endian
        return TypeConversion.unsignedShortToInt(
                TypeConversion.byteArrayToShort(
                        this.deviceClient.read(reg.bVal, 2), ByteOrder.LITTLE_ENDIAN));
    }

    public int getEnableFlags() {
        return readUnsignedByte(Register.ENABLE);
    }

    public void enable(EnableFlags ... flags) {
        byte enabled = (byte)getEnableFlags();
        for(EnableFlags f : flags) {
            enabled |= f.bVal;
        }
        write8(Register.ENABLE, enabled);
    }

    public void disable(EnableFlags ... flags) {
        byte enabled = (byte)getEnableFlags();
        for(EnableFlags f : flags) {
            enabled &= ~f.bVal;
        }
        write8(Register.ENABLE, enabled);
    }

    public void enableOnly(EnableFlags ... flags) {
        byte enabled = 0;
        for(EnableFlags f : flags) {
            enabled |= f.bVal;
        }
        write8(Register.ENABLE, enabled);
    }

    public void setAdcIntegTime(double time) {
        // 178ms required for full resolution,
        // however decent resolution possible in slower times
        double steps = 256.0 - time / TIME_MULT;
        byte r = (byte)Math.max(0, Math.min(255, steps));
        Log.v(TAG,String.format("Writing %02x to ATIME", r));
        write8(Register.ATIME, r);
    }

    public double getAdcIntegTime() {
        return (double)readUnsignedByte(Register.ATIME) * TIME_MULT;
    }


    public void setWaitTime(double time) {
        double steps = 256.0 - time / TIME_MULT;
        byte r = (byte)Math.max(0, Math.min(255, steps));
        Log.v(TAG,String.format("Writing %02x to WTIME", r));
        write8(Register.WTIME, r);
    }

    public double getWaitTime() {
        return (double)readUnsignedByte(Register.WTIME) * TIME_MULT;
    }

    public void setAdcGain(AdcGain g) {
        int r = readUnsignedByte(Register.CONTROL) & 0xCC;
        r |= g.bVal;
        Log.v(TAG,String.format("Writing %02x to CONTROL", r));
        write8(Register.CONTROL, (byte)r);
    }

    public void setProxGain(ProxGain g) {
        int r = readUnsignedByte(Register.CONTROL) & 0xC3;
        r |= g.bVal<<2;
        Log.v(TAG,String.format("Writing %02x to CONTROL", r));
        write8(Register.CONTROL, (byte)r);
    }

    public void setLedLDrive(LedLDrive drive) {
        int r = readUnsignedByte(Register.CONTROL) & 0x0F;
        r |= drive.bVal<<6;
        Log.v(TAG,String.format("Writing %02x to CONTROL", r));
        write8(Register.CONTROL, (byte)r);
    }

    public void setLedBoost(LedBoost boost) {
        // I've not seen any benefit from setting boost, but probably
        // because I've only tried LDrive = 100mA
        int r = readUnsignedByte(Register.CONFIG2) & 0xC0 | 0x01;
        r |= boost.bVal<<4;
        Log.v(TAG,String.format("Writing %02x to CONFIG2", r));
        write8(Register.CONFIG2, (byte)r);
    }

    public void setProxPulse(LedPPLen len, int count) {
        count = Math.max(0, Math.min(63, count));
        int r = (len.bVal<<6) | count;
        Log.v(TAG,String.format("Writing %02x to PPULSE", r));
        write8(Register.PPULSE, (byte)r);
    }

    public int readId() {
        // For device check
        return readUnsignedByte(Register.ID);
    }

    public int readAmbientLightLevel() {
        return readUnsignedShort(Register.CDATAL);
    }

    public int readRedData() {
        return readUnsignedShort(Register.RDATAL);
    }

    public int readGreenData() {
        return readUnsignedShort(Register.GDATAL);
    }

    public int readBlueData() {
        return readUnsignedShort(Register.BDATAL);
    }

    public int readProximityData() {
        return readUnsignedByte(Register.PDATA);
    }


}
