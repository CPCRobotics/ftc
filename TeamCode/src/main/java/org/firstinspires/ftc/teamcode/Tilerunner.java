package org.firstinspires.ftc.teamcode;

import android.support.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.hardware.AdafruitADPS9960;
import org.firstinspires.ftc.teamcode.hardware.ProximitySensor;
import org.firstinspires.ftc.teamcode.util.DCMotorGroup;
import org.firstinspires.ftc.teamcode.util.ServoGroup;
import org.firstinspires.ftc.teamcode.util.SpeedController;
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
    private static final String ROBOT_VERSION = "0.1.1";

    // Hardware
    private static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
    private double ticksPerInch = 0;

    // Thresholds
    private static final double THRESHOLD_INCHES = 4;
    private static final double THRESHOLD_HEADING_DEG = 360;
    public static final double MOTOR_DEADZONE = 0.2; // range [0,1]

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
    private DcMotor lightRed = new NullDcMotor();
    private DcMotor lightGreen = new NullDcMotor();

    // Aligns the glyph in the 4th row to prevent it from toppling
    private Servo holderLeft = new NullServo();
    private Servo holderRight = new NullServo();

    private boolean usingEasyLift = false;

    private Double currentPosition = null;
    public Servo jewelWhacker;
    private Servo kicker = new NullServo();

    private BNO055IMU imu;
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
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        // Create a motor pair when manipulating both wheels at the same time
        motorPair = new DCMotorGroup(Arrays.asList(leftMotor, rightMotor));

        // Lights
        lightRed = new DCMotorGroup(Arrays.asList(
                createDcMotor(hardwareMap, "lightr1"),
                createDcMotor(hardwareMap, "lightr2")));
        lightGreen = new DCMotorGroup(Arrays.asList(
                createDcMotor(hardwareMap, "lightg1"),
                createDcMotor(hardwareMap, "lightg2")
        ));

        // Set up ticks/in
        ticksPerInch = leftMotor.getMotorType().getTicksPerRev() / WHEEL_CIRCUMFERENCE;


        // Claw motor
        clawMotor = createDcMotor(hardwareMap, "claw");


        // Kickers are grouped to behave the same
        Servo kicker1 = getHardware(Servo.class, hardwareMap, "kicker", new NullServo());
        Servo kicker2 = getHardware(Servo.class, hardwareMap, "kicker2", new NullServo());
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


        //  IMU
        imu = new NullBNO055IMU();
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

    // Utility Commands

    /**
     * @return the speed to run according to the goal distance left
     */
    private double calculateSpeed(double dist, double threshold) {

        // Slow down when the distance to the target is less than the threshold
        return Math.min(1, Math.max(MOTOR_DEADZONE, dist / threshold) );
    }

    /**
     * Moves the robot a specified distance.
     */
    public void move(BusyWaitHandler waitHandler, double powerMultiplier, double inches) {
        int ticks = (int)(ticksPerInch * inches);

        motorPair.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPair.setTargetPosition( ticks );
        motorPair.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorPair.setPower(powerMultiplier);

        while (leftMotor.isBusy() && waitHandler.isActive()) {
            double currentPower = calculateSpeed(
                    Math.abs(motorPair.getTargetPosition() - motorPair.getCurrentPosition()),
                    THRESHOLD_INCHES / ticksPerInch);
            Twigger.getInstance()
                    .addLine(".move()")
                    .addData("mode", leftMotor.getMode())
                    .addData("target", leftMotor.getTargetPosition())
                    .addData("pos", leftMotor.getCurrentPosition());

            motorPair.setPower(currentPower * powerMultiplier);
        }

        Twigger.getInstance()
                .update()
                .remove(".move()");
    }

    private double distanceDegrees(double start, double end, double direction) {
        double dist = (direction > 0) ? start - end : end - start;
        if (dist < 0)
            dist += 360;
        return dist;
    }

    /**
     * Get the angle the robot is at range [0-360)
     */
    private double getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 180;
    }

    /**
     * Rotates the robot at a specified angle.
     *1
     * WARNING: Here be dragons! This is the result of over two weeks of debugging
     * and a tired developer that's too afraid to change it any more, lets it breaks
     * again.
     */
    public void turn(BusyWaitHandler waitHandler, double power, double angle)
            throws InterruptedException {

        // The current position is calibrated on the first .turn() call
        if (currentPosition == null)
            currentPosition = getHeading();

        Twigger.getInstance().sendOnce("Arguments: power " + power + ", angle " + angle);

        motorPair.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPair.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Calculate the distance within a range of 0°-360°
        double destination = (currentPosition - angle) % 360;
        // The % operator doesn't convert a negative value to a positive value;
        //   do this manually.
        if (destination < 0)
            destination += 360;

        // Get the product of all signs of both numbers, i.e. positive if
        //  both numbers are the same sign, and negative otherwise
        int direction = (int) (Math.signum(power) * Math.signum(angle));
        power = Math.abs(power);

        double delta = distanceDegrees(getHeading(), destination, direction);

        Twigger.getInstance().sendOnce("Calculations: dest " + destination + ", delta " + delta);
        while (delta > TURN_THRESHOLD_DEG && delta < (360 - TURN_THRESHOLD_DEG) && waitHandler.isActive()) {

            // Update the current speed
            double currentPower = power * calculateSpeed(delta, THRESHOLD_HEADING_DEG);
            currentPower = Math.max(MOTOR_DEADZONE, currentPower);
            currentPower *= direction;

            leftMotor.setPower(currentPower);
            rightMotor.setPower(-currentPower);

            Twigger.getInstance().addLine(".turn()")
                    .addData("power", currentPower)
                    .addData("delta", delta)
                    .addData("dest", destination)
                    .addData("curr", getHeading());

            Thread.sleep(1);
            delta = distanceDegrees(getHeading(), destination, direction);
        }

        motorPair.setPower(0);
        motorPair.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Thread.sleep(100); // Wait for momentum to finish

        delta = distanceDegrees(getHeading(), destination, direction);

        currentPosition = destination;


        // Correct turn if too bad after momentum is gone
        if (Math.min(delta, 360-delta) > TURN_THRESHOLD_DEG) {
            // Calculate shortest distance to be corrected
            double correctionSpeed = delta < 360-delta ? delta : delta-360;

            Twigger.getInstance().sendOnce("Delta (" + delta + ") Too big. Correcting by " +
                    correctionSpeed);
            turn(waitHandler, power/4, correctionSpeed);
        }


        Twigger.getInstance()
                .update()
                .remove(".turn()");

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

    private boolean isHoldingGlyph() {
        return proximitySensor.getDistance(DistanceUnit.MM) <= HOLDING_GLYPH_DIST_MM;
    }

    public void setRed(boolean isOn) {
        lightRed.setPower(isOn ? LIGHT_MOTOR_SPEED : 0);
    }

    public void setGreen(boolean isOn) {
        lightGreen.setPower(isOn ? LIGHT_MOTOR_SPEED : 0);
    }

    public void setLights(boolean red, boolean green) {
        setRed(red);
        setGreen(green);
    }

    public boolean glyphDetected() {
        return proximitySensor.getDistance(DistanceUnit.MM) > HOLDING_GLYPH_DIST_MM;
    }
}

