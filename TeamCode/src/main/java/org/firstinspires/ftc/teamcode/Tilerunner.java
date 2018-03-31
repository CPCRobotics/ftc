package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.hardware.AdafruitADPS9960;
import org.firstinspires.ftc.teamcode.hardware.ProximitySensor;
import org.firstinspires.ftc.teamcode.util.DCMotorGroup;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.util.RobotType;
import org.firstinspires.ftc.teamcode.util.ServoGroup;
import org.firstinspires.ftc.teamcode.util.SpeedController;
import org.firstinspires.ftc.teamcode.util.VirtualCompass;
import org.firstinspires.ftc.teamcode.util.nulls.NullBNO055IMU;
import org.firstinspires.ftc.teamcode.util.nulls.NullDcMotor;
import org.firstinspires.ftc.teamcode.util.nulls.NullDigitalChannel;
import org.firstinspires.ftc.teamcode.util.nulls.NullProximitySensor;
import org.firstinspires.ftc.teamcode.util.nulls.NullServo;
import org.firstinspires.ftc.teamcode.twigger.Twigger;
import org.firstinspires.ftc.teamcode.util.BusyWaitHandler;

import java.io.File;
import java.util.Arrays;

/**
 * All the hardware and its utility methods
 */
public class Tilerunner {
    private static final String ROBOT_VERSION = "0.1.2";

    // Hardware
    private static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    private double ticksPerInch = 0;

    // Thresholds
    public static final double MOTOR_DEADZONE = 0.05; // range [0,1]

    private static final double TURN_THRESHOLD_DEG = 5;

    private static final double HOLDING_GLYPH_DIST_MM = 10;

    private boolean ignoreSoftwareLimits = false;

    private static final int LIFT_MOTOR_MIN = 10;
    private static final int LIFT_MOTOR_MAX = 2480; // True max: 2518
    private static final int LIFT_LOW_POSITION = 200;

    private static final double LIGHT_MOTOR_SPEED = 1;

    private static final int NEAR_LIFT_POSITION_THRESHOLD = 150;

    private static final double ZERO_LIFT_POWER = 0.4;

    // Wheels
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor motorPair;

    private DcMotor clawMotor;
    private DcMotor liftMotor;
    private DigitalChannel liftSensorLow;
    private DigitalChannel liftSensorHigh;

    // According to Game Rules, lights can not be controlled via Digital Output,
    // but they can still be controlled via DC Motors.
    private DcMotor lightRedL = new NullDcMotor();
    private DcMotor lightRedR = new NullDcMotor();
    private DcMotor lightGreenL = new NullDcMotor();
    private DcMotor lightGreenR = new NullDcMotor();

    // Aligns the glyph in the 4th row to prevent it from toppling
    private Servo holderLeft = new NullServo();
    private Servo holderRight = new NullServo();

    private boolean usingEasyLift = false;

    private Double currentPosition = null;
    public Servo jewelWhacker;
    private Servo kicker = new NullServo();

    public VirtualCompass compass;
    private ProximitySensor proximitySensor;

    private final SpeedController leftController = new SpeedController();
    private final SpeedController rightController = new SpeedController();

    private OpmodeType selectedOpmode;

    public enum OpmodeType {
        TELEOP, AUTONOMOUS
    }

    public enum CryptoboxRow {
        LOWEST(LIFT_MOTOR_MIN),
        LOWER(LIFT_MOTOR_MIN + (LIFT_MOTOR_MAX - LIFT_MOTOR_MIN) / 3),
        HIGHER(LIFT_MOTOR_MIN + (LIFT_MOTOR_MAX - LIFT_MOTOR_MIN) * 2 / 3),
        HIGHEST(LIFT_MOTOR_MAX);

        public final int liftPosition;
        CryptoboxRow(int liftPosition) {
            this.liftPosition = liftPosition;
        }

        public CryptoboxRow nextHigherRow() {
            switch (this) {
                case LOWEST: return LOWER;
                case LOWER: return HIGHER;
                default: return HIGHEST;
            }
        }

        public CryptoboxRow nextLowerRow() {
            switch (this) {
                case HIGHEST: return HIGHER;
                case HIGHER: return LOWER;
                default: return LOWEST;
            }
        }

        public int distanceFrom(int liftPosition) {
            return Math.abs(this.liftPosition - liftPosition);
        }

        public static CryptoboxRow getNearestRow(int liftPosition) {
            int nearestDistance = LIFT_MOTOR_MAX + 100; // Impossible for the dist to be bigger
            CryptoboxRow nearestRow = null;

            for (CryptoboxRow row : values()) {
                int dist = row.distanceFrom(liftPosition);
                if (dist < nearestDistance) {
                    nearestDistance = dist;
                    nearestRow = row;
                }
            }

            return nearestRow;
        }

        public static CryptoboxRow selectNextRow(Tilerunner tilerunner, boolean goingUp) {
            int liftPosition = tilerunner.liftMotor.getCurrentPosition();

            CryptoboxRow row = getNearestRow(liftPosition);

            // If the closest row is still far away, go to that row.
            if (((goingUp && liftPosition < row.liftPosition) ||
                 (!goingUp && liftPosition > row.liftPosition)) &&
                    row.distanceFrom(liftPosition) > NEAR_LIFT_POSITION_THRESHOLD) {
                return row;
            } else {
                return goingUp ? row.nextHigherRow() : row.nextLowerRow();
            }
        }
    }

    /// ----------Init Utility Methods----------

    /**
     * Generic hardware access method
     */
    private<T> T getHardware(Class<? extends T> classObj, HardwareMap hardwareMap, String deviceName,
                            @NonNull T nullObject) {
        try {
            return hardwareMap.get(classObj, deviceName);
        } catch (IllegalArgumentException e) {
            Twigger.getInstance().sendOnce("WARN: device '" + deviceName + "' missing.");
            return nullObject;
        }
    }

    /**
     * Getting DcMotors use a different method than other hardware
     */
    private DcMotor createDcMotor(HardwareMap hardwareMap, String motorName){
        DcMotor motor;
        try{
            motor=hardwareMap.dcMotor.get(motorName);
        } catch (IllegalArgumentException e) {
            Twigger.getInstance().sendOnce("WARN: " + motorName + " motor missing: "+ e.getMessage());
            return new NullDcMotor();
        }
        motor.setDirection(DcMotor.Direction.FORWARD); // Forward is the default direction
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    /* Initialize standard Hardware interfaces */
    public void init( HardwareMap hardwareMap, Telemetry telemetry, OpmodeType opMode) {

        selectedOpmode = opMode;

        ElapsedTime initTime = new ElapsedTime();

        Twigger.getInstance().init(telemetry);

        // Show build version to keep track of logging and verify the code is up-to-date
        Twigger.getInstance().sendOnce("Robot build v" + ROBOT_VERSION);

        //  Drive Motors
        leftMotor   = createDcMotor(hardwareMap, "left_drive");
        rightMotor  = createDcMotor(hardwareMap, "right_drive");

        // The competition and test bots have the tilerunner hooked up in reverse.
        if (RobotType.detect(hardwareMap) == RobotType.COMPETITION) {
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Create a motor pair when manipulating both wheels at the same time
        motorPair = new DCMotorGroup(Arrays.asList(leftMotor, rightMotor));

        // Lights
        lightRedL = createDcMotor(hardwareMap, "lightr1");
        lightRedR = createDcMotor(hardwareMap, "lightr2");
        lightGreenL = createDcMotor(hardwareMap, "lightg1");
        lightGreenR = createDcMotor(hardwareMap, "lightg2");

        // Set up ticks/in
        ticksPerInch = leftMotor.getMotorType().getTicksPerRev() / WHEEL_CIRCUMFERENCE;


        // Claw motor
        clawMotor = createDcMotor(hardwareMap, "claw");


        // Kickers are grouped to behave the same
        Servo kicker1 = getHardware(Servo.class, hardwareMap, "kicku", new NullServo());
        Servo kicker2 = getHardware(Servo.class, hardwareMap, "kickd", new NullServo());
        kicker = new ServoGroup(kicker1, kicker2);

        if (opMode == OpmodeType.AUTONOMOUS)
            primeKicker(); // Kicker can't be primed in TeleOp


        // Jewel Whacker
        jewelWhacker = getHardware(Servo.class, hardwareMap, "whacker", new NullServo());

        if (opMode == OpmodeType.AUTONOMOUS)
            retractJewelWhacker(); // Whacker can't be retracted in TeleOp


        // LIFT devices
        try {
            liftMotor = createDcMotor(hardwareMap, "lift");

            liftSensorLow = hardwareMap.digitalChannel.get("lift_low");
            liftSensorLow.setMode(DigitalChannel.Mode.INPUT);
            liftSensorHigh = hardwareMap.digitalChannel.get("lift_high");
            liftSensorHigh.setMode(DigitalChannel.Mode.INPUT);
        } catch (IllegalArgumentException e) {
            liftMotor = new NullDcMotor();
            liftSensorLow = new NullDigitalChannel();
            liftSensorHigh = new NullDigitalChannel();
        }


        //  Compass (IMU)
        BNO055IMU imu = new NullBNO055IMU();
        if (opMode == OpmodeType.AUTONOMOUS) {
            try {
                // Set up the parameters with which we will use our IMU. Note that integration
                // algorithm here just reports accelerations to the logcat log; it doesn't actually
                // provide positional information.
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
                parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parameters.loggingEnabled      = true;
                parameters.loggingTag          = "IMU";
                parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

                imu = hardwareMap.get(BNO055IMU.class, "imu");

                // Check if IMU Calibration exists
                File file = AppUtil.getInstance().getSettingsFile(parameters.calibrationDataFile);
                if (!file.exists())
                    Twigger.getInstance().sendOnce("WARN: Calibration File Doesn't Exist");


                Twigger.getInstance().sendOnce("Initializing IMU");
                if (!imu.initialize(parameters))
                    throw new IllegalArgumentException(); // IMU can't be initialized; Bad News!

                Twigger.getInstance().sendOnce("Initialized IMU");
            } catch (IllegalArgumentException e) {
                Twigger.getInstance().sendOnce("WARN: IMU Sensor Missing");
            }
        }
        compass = new VirtualCompass(imu);


        // Proximity Sensor
        proximitySensor = getHardware(AdafruitADPS9960.class, hardwareMap, "range",
                new NullProximitySensor());


        // Glyph Holder is only used in TeleOp
        if (opMode == OpmodeType.TELEOP) {
            holderLeft = getHardware(Servo.class, hardwareMap, "holderL",
                    holderLeft);
            holderRight = getHardware(Servo.class, hardwareMap, "holderR",
                    holderRight);
        }


        Twigger.getInstance()
                .sendOnce("Tilerunner initialized in " + initTime.seconds() + " seconds.")
                .update();

        setLights(true, true);
    }

    /**
     * Sets lift to the lowest point possible and resets the encoder
     */
    public void zeroLift(BusyWaitHandler waitHandler) {
        try {
            while (!(isLiftAtLowPoint()) && waitHandler.isActive()) { // Wait until the channel throws a positive
                liftMotor.setPower(-ZERO_LIFT_POWER);
                Thread.sleep(1);
            }

        } catch(InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isLiftAtLowPoint() {
        return !liftSensorLow.getState();
    }

    public boolean isLiftAtHighPoint() {
        return !liftSensorHigh.getState();
    }

    public void setIgnoreSoftwareLimits(boolean ignoreSoftwareLimits) {
        this.ignoreSoftwareLimits = ignoreSoftwareLimits;
    }

    public int getElevatorPos() {
        return liftMotor.getCurrentPosition();
    }

    /**
     * Moves the robot a specified distance.
     */
    public void move(BusyWaitHandler waitHandler, double destInches, PIDController pid) throws InterruptedException {
        // Calculate required ticks
        int destTicks = (int)(ticksPerInch * destInches);

        // Set up motors to stop at destination
        motorPair.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPair.setTargetPosition( destTicks );
        motorPair.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (leftMotor.isBusy() && waitHandler.isActive()) {
            double
                    currPos = motorPair.getCurrentPosition() / ticksPerInch,
                    err = destInches - currPos,
                    currPower = pid.get(err);

            // Ensure the motors are powerful enough to move the bot
            currPower = Math.max(MOTOR_DEADZONE, Math.abs(currPower)) * Math.signum(currPower);

            motorPair.setPower(currPower);

            Twigger.getInstance().addLine(".move()")
                    .addData("pos", currPos)
                    .addData("target", destInches)
                    .addData("err", err)
                    .addData("pow", currPower);

        }

        Twigger.getInstance()
                .update()
                .remove(".move()");
    }

    public void move(BusyWaitHandler waitHandler, double power, double destInches) throws InterruptedException {

        // If power is negative (it shouldn't), flip the sign to the destination
        if (power < 0) {
            Twigger.getInstance()
                    .sendOnce("WARN: .move() received negative power (" + power + ").");
            power *= -1;
            destInches *= -1;
        }

        move(waitHandler, destInches, new PIDController(-power, power, 0.02, 0, 0));
    }

    /**
     * Rotates the robot at a specified angle.
     *1
     * WARNING: Here be dragons! This is the result of over two weeks of debugging
     * and a tired developer that's too afraid to change it any more, lets it breaks
     * again.
     */
    public void turn(BusyWaitHandler waitHandler, double angle, PIDController pid)
            throws InterruptedException {

        // Turn algorithm uses positive angles to turn CCW. .turn() promises
        // positive angles turns CW instead.
        angle *= -1;

        if (currentPosition == null)
            currentPosition = compass.getAngle();
        final double dest = currentPosition + angle;

        motorPair.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPair.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double error;
        while (waitHandler.isActive() && Math.abs(error = dest - compass.getAngle()) > TURN_THRESHOLD_DEG) {
            double currentPower = pid.get(error);
            // ensure movement is powerful enough
            currentPower = Math.max(MOTOR_DEADZONE, Math.abs(currentPower)) * Math.signum(currentPower);

            leftMotor.setPower(-currentPower);
            rightMotor.setPower(currentPower);

            Twigger.getInstance().addLine(".turn()")
                    .addData("power", currentPower)
                    .addData("dest", dest)
                    .addData("curr", compass.getAngle())
                    .addData("err", error);

            Thread.sleep(1);
        }

        motorPair.setPower(0);
        motorPair.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Thread.sleep(100); // Wait for momentum to finish

        currentPosition = dest;

        Twigger.getInstance()
                .update()
                .remove(".turn()");

    }

    public void turn(BusyWaitHandler waitHandler, double power, double angle) throws InterruptedException {
        // If power is negative, shift it to angles.
        if (power < 0) {
            angle *= -1;
            power *= -1;
        }

        turn(waitHandler, angle, new PIDController(-power, power, 0.1, 0, 0));
    }

    /**
     * Removes the glyph from the robot
     */
    public void ejectGlyph(BusyWaitHandler waitHandler) throws InterruptedException {
        try {
            ejectGlyph(1);
            longSleep(waitHandler, 1000);
        } finally {
            primeKicker();
        }
    }

    public void ejectGlyph(double power, boolean launchKicker) {
        clawMotor.setPower(power);
        if (launchKicker)
            launchKicker();
    }

    public void ejectGlyph(double power) {
        ejectGlyph(power, true);
    }

    private boolean autoDeployWaiting = true;
    public void grabGlyph(double power) {
        clawMotor.setPower(-power);
        primeKicker();

        // Put holders up if we're moving for the first time
        if (autoDeployWaiting && selectedOpmode == OpmodeType.TELEOP && power != 0) {
            setHolderUp();
            autoDeployWaiting = false;
        }
    }

    /**
     * Sets the jewel whacker out
     */
    public void activateJewelWhacker(BusyWaitHandler waitHandler) throws InterruptedException {
        try {
            jewelWhacker.setPosition(.75);
            longSleep(waitHandler, 750); // Let the tape straighten out
        } finally {
            jewelWhacker.setPosition(0);
        }
    }

    /**
     * Set the jewel whacker back up
     */
    public void retractJewelWhacker() {
        jewelWhacker.setPosition(1);
    }

    /**
     * Set how fast the lift should move
     */
    public void setLiftPower(double power) {

        // Do nothing if EasyLift is busy
        if (usingEasyLift && power == 0)
            return;

        usingEasyLift = false;


        // Slow down when near edges
        if (liftMotor.getCurrentPosition() <= LIFT_LOW_POSITION)
            power *= .5;

        if (power > 0 && !isLiftAtHighPoint()) {
            // allow motor to go upwards, but limit at distance above zero
            if (ignoreSoftwareLimits)
                liftMotor.setTargetPosition(99999999);
            else
                liftMotor.setTargetPosition(LIFT_MOTOR_MAX);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (power < 0 && !isLiftAtLowPoint()) {
            // allow motor to go downwards, but limit at zero
            liftMotor.setTargetPosition(LIFT_MOTOR_MIN);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (isLiftAtLowPoint()) {
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            power = 0;
        } else {
            int desired = Math.max(0, Math.min(LIFT_MOTOR_MAX, liftMotor.getCurrentPosition()));
            if (ignoreSoftwareLimits)
                desired = Math.max(0, liftMotor.getCurrentPosition());

            liftMotor.setTargetPosition(desired);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            power = 0;
        }

        liftMotor.setPower(power);
        Twigger.getInstance().update();
    }

    // Motors
    public void setMotors(double speedLeft, double speedRight) {
        leftMotor.setPower(leftController.get(speedLeft));
        rightMotor.setPower(rightController.get(speedRight));
    }

    public void setMotors(double speed) {
        setMotors(speed, speed);
    }

    public void changeLiftPosition(boolean goingUp) {
        changeLiftPosition(CryptoboxRow.selectNextRow(this, goingUp));
    }

    public void changeLiftPosition(CryptoboxRow row) {
        liftMotor.setTargetPosition(row.liftPosition);
        liftMotor.setPower(1);
        usingEasyLift = true;
    }

    /**
     * Sleeps until interrupted or op mode no longer active
     */
    public void longSleep(BusyWaitHandler waitHandler, int millis) throws InterruptedException {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.milliseconds() < millis) {
            Thread.sleep(10);
            if (!waitHandler.isActive()) {
                throw new InterruptedException("OpMode Timed Out");
            }
        }
    }

    private void primeKicker() {
        kicker.setPosition(1);
    }

    private void launchKicker() {
        kicker.setPosition(0);
    }

    /**
     * Sets position of glyph holder
     *
     * @param val 0 is down; 1 is up
     */
    private void setGlyphHolder(double val) {
        holderLeft.setPosition(1 - val); // This servo needs to be inverted
        holderRight.setPosition(val);
    }

    public void setHolderUp() {
        setGlyphHolder(1);
    }

    public void setHolderDown() {
        setGlyphHolder(0);
    }

    private void setLight(DcMotor light, boolean isOn) {
        light.setPower(isOn ? LIGHT_MOTOR_SPEED : 0);
    }

    public void setRed(boolean left, boolean right) {
        setLight(lightRedL, left);
        setLight(lightRedR, right);
    }

    public void setRed(boolean isOn) {
        setRed(isOn, isOn);
    }

    public void setGreen(boolean left, boolean right) {
        setLight(lightGreenL, left);
        setLight(lightGreenR, right);
    }

    public void setGreen(boolean isOn) {
        setGreen(isOn, isOn);
    }

    public void setLights(boolean red, boolean green) {
        setRed(red);
        setGreen(green);
    }

    public void setLights(boolean isOn) {
        setLights(isOn, isOn);
    }

    public boolean isHoldingGlyph() {
        return proximitySensor.getDistance(DistanceUnit.MM) <= HOLDING_GLYPH_DIST_MM;
    }
}

