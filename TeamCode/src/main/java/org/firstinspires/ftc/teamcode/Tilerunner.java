package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.nulls.NullBNO055IMU;
import org.firstinspires.ftc.teamcode.nulls.NullDcMotor;
import org.firstinspires.ftc.teamcode.nulls.NullDigitalChannel;
import org.firstinspires.ftc.teamcode.nulls.NullServo;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

import java.io.File;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left drive motor:        "left_drive"
 * Motor channel:  Right drive motor:       "right_drive"

 */
@SuppressWarnings("WeakerAccess")
public class Tilerunner
{
    /* Public OpMode members. */

    /*
        wheel-revs-per-robot-rev = robot-base-diameter / wheel-circumference
        robot-base-diameter = 15"
        wheel-diameter = 4"
        15" / 4" = 3.75
     */
    public static final double WHEEL_REVS_PER_ROBOT_REV = 3.75;
    // Ticks that make up the circumference of the wheel
    private static final int TICKS_PER_REVOLUTION = 1120;
    // 4" * pi = 12.5663"
    private static final double WHEEL_CIRCUMFERENCE = 12.5663;
    private static final double THRESHOLD_TICKS = Tilerunner.TICKS_PER_REVOLUTION;
    private static final double THRESHOLD_HEADING = 180;
    public static final double MOTOR_DEADZONE = 0.2;

    private static final double TURN_THRESHOLD = 2.5; // degrees

    private static final int DISTANCE_REMOVE_GLYPH = 10; // TODO get needed ticks to remove glyph

    public DcMotor  clawMotor;
    public DcMotor  leftMotor;
    public DcMotor  rightMotor;
    public DcMotor motorPair;
    public DcMotor liftMotor;
    public DigitalChannel liftSensorLow;
    public DigitalChannel liftSensorHigh;

    private boolean liftOverride = false;
    public static final int LIFT_MOTOR_MIN = 10; // True max: 3320
    public static final int LIFT_MOTOR_MAX = 3300; // True max: 3320
    public static final boolean INVERTED_LIFT_SENSOR = true;
    public static final int LIFT_LOW_POSITION = 200;

    private boolean usingEasyLift = false;

    private Double currentPosition = null;


    Servo jewelWhacker;
    public Servo kicker = new NullServo();

    BNO055IMU imu;

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

        public static CryptoboxRow selectNextRow(Tilerunner tilerunner, boolean goingUp) {
            int liftPosition = tilerunner.liftMotor.getCurrentPosition();

            if (goingUp) {
                for (CryptoboxRow row : values()) {
                    if (liftPosition > row.liftPosition)
                        return row.nextHigherRow();
                }
                return HIGHEST;
            } else {
                List<CryptoboxRow> vals = Arrays.asList(values());
                Collections.reverse(vals);
                for (CryptoboxRow row : vals) {
                    if (liftPosition < row.liftPosition)
                        return row.nextLowerRow();
                }
                return LOWEST;
            }
        }
    }

    private DcMotor createDcMotor(HardwareMap hardwareMap, String motorName){
        DcMotor motor;
        try{
            motor=hardwareMap.dcMotor.get(motorName);
        } catch (IllegalArgumentException e) {
            Twigger.getInstance().sendOnce("WARN: " + motorName + " motor missing: "+ e.getMessage());
            return new NullDcMotor();
        }
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    /* Initialize standard Hardware interfaces */
    public void init( HardwareMap hardwareMap, Telemetry telemetry ) {

        Twigger.getInstance().setTelemetry(telemetry);

        // Define and Initialize Motors
        leftMotor   = createDcMotor(hardwareMap, "left_drive");
        rightMotor  = createDcMotor(hardwareMap, "right_drive");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // Create a motor pair when manipulating both leftMotor and rightMotor
        motorPair = new DCMotorGroup(Arrays.asList(leftMotor, rightMotor));


        clawMotor = createDcMotor(hardwareMap, "claw");
        try {
            kicker = hardwareMap.servo.get("servo");
        } catch (IllegalArgumentException e) {
            Twigger.getInstance().sendOnce("WARN: Kicker doesn't exist");
        }

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

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            // Check if IMU Calibration exists
            File file = AppUtil.getInstance().getSettingsFile(parameters.calibrationDataFile);
            if (!file.exists())
                Twigger.getInstance().sendOnce("WARN: Calibration File Doesn't Exist");

            Twigger.getInstance().sendOnce("Initializing IMU");

            if (!imu.initialize(parameters))
                throw new IllegalArgumentException();

            Twigger.getInstance().sendOnce("Initialized IMU");
        } catch (IllegalArgumentException e) {
            imu = new NullBNO055IMU();
            Twigger.getInstance().sendOnce("WARN: IMU Sensor Missing");
        }

        initWhacker(hardwareMap, telemetry);

        try {
            initLift(hardwareMap, telemetry);
        } catch (IllegalArgumentException e) {
            liftMotor = new NullDcMotor();
            liftSensorLow = new NullDigitalChannel();
            liftSensorHigh = new NullDigitalChannel();
        }

        Twigger.getInstance().update();
    }

    void initWhacker(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            jewelWhacker = hardwareMap.servo.get("whacker");
            jewelWhacker.setPosition(1);
        } catch (IllegalArgumentException e) {
            jewelWhacker = new NullServo();
            Twigger.getInstance().sendOnce("WARN: Jewel Whacker Missing");
        }
    }

    void initLift(HardwareMap hardwareMap, Telemetry telemetry) throws IllegalArgumentException {
        liftMotor = createDcMotor(hardwareMap, "lift");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftSensorLow = hardwareMap.digitalChannel.get("lift_low");
        liftSensorLow.setMode(DigitalChannel.Mode.INPUT);
        liftSensorHigh = hardwareMap.digitalChannel.get("lift_high");
        liftSensorHigh.setMode(DigitalChannel.Mode.INPUT);
    }

    public void zeroLift() {
        // only in autonomous init
        try {
            while (!(isLiftAtLowPoint())) { // Wait until the channel throws a positive
                liftMotor.setPower(-0.15);
                Thread.sleep(1);
            }
        } catch(InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private boolean isLiftAtLowPoint() {
        return INVERTED_LIFT_SENSOR != liftSensorLow.getState();
    }

    private boolean isLiftAtHighPoint() {
        return INVERTED_LIFT_SENSOR != liftSensorHigh.getState();
    }

    public double getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 180;
    }

    // Utility Commands

    /**
     *
     * @return the speed to run according to the goal distance left
     */
    private double calculateSpeed(double dist, double threshold) {

        // Slow down when the distance to the target is less than the threshold
        return Math.min(1, Math.max(MOTOR_DEADZONE, dist / threshold) );
    }

    public void move(BusyWaitHandler waitHandler, double power, double inches) {
        int ticks = (int)(Tilerunner.TICKS_PER_REVOLUTION * inches / Tilerunner.WHEEL_CIRCUMFERENCE);

        motorPair.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPair.setTargetPosition( ticks );
        motorPair.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorPair.setPower(power);

        while (leftMotor.isBusy() && waitHandler.isActive()) {
            double currentPower = calculateSpeed(
                    motorPair.getTargetPosition() -motorPair.getCurrentPosition(),
                    THRESHOLD_TICKS);
            Twigger.getInstance()
                    .addLine(".move()")
                    .addData("mode", leftMotor.getMode())
                    .addData("target", leftMotor.getTargetPosition())
                    .addData("pos", leftMotor.getCurrentPosition());

            motorPair.setPower(currentPower);
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
     * Rotates the robot at a specified angle.
     */
    public void turn(BusyWaitHandler waitHandler, double power, double angle)
            throws InterruptedException {

        if (currentPosition == null)
            currentPosition = getHeading();

        Twigger.getInstance().sendOnce("Arguments: power " + power + ", angle " + angle);

        motorPair.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPair.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double destination = (currentPosition - angle) % 360;
        if (destination < 0)
            destination += 360;

        int direction = (int) (Math.signum(power) * Math.signum(angle));
        power = Math.abs(power);

        double delta = distanceDegrees(getHeading(), destination, direction);

        Twigger.getInstance().sendOnce("Calculations: dest " + destination + ", delta " + delta);
        while (delta > TURN_THRESHOLD && waitHandler.isActive()) {

            double currentPower = power * calculateSpeed(delta, THRESHOLD_HEADING);
            currentPower = Math.max(MOTOR_DEADZONE, currentPower);
            currentPower *= direction;

            leftMotor.setPower(currentPower);
            rightMotor.setPower(-currentPower);

            Twigger.getInstance().addLine(".turn()")
                    .addData("power", currentPower)
                    .addData("delta", delta)
                    .addData("dest2", destination)
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
        if (delta > TURN_THRESHOLD) {
            turn(waitHandler, power/2, delta*direction);
        }

        Twigger.getInstance()
                .update()
                .remove(".turn()");

    }

    public void ejectGlyph(BusyWaitHandler waitHandler) throws InterruptedException {
        try {
            clawMotor.setPower(1);
            longSleep(waitHandler, 1000);
        } finally {
            clawMotor.setPower(0);
        }
    }

    public void activateJewelWhacker(BusyWaitHandler waitHandler) throws InterruptedException {
        try {
            jewelWhacker.setPosition(.75);
            longSleep(waitHandler, 750); // Let the tape straighten out
        } finally {
            jewelWhacker.setPosition(0);
        }
    }

    public void retractJewelWhacker() {
        jewelWhacker.setPosition(1);
    }

    public boolean getLiftOverride() {
        return liftOverride;
    }

    public void setLiftPower(double power) {

        double liftPowerMultiplier;
        if (liftMotor.getCurrentPosition() <= LIFT_LOW_POSITION)
            liftPowerMultiplier = 0.5;
        else
            liftPowerMultiplier = 1.0;

        if (power > 0 && !isLiftAtHighPoint()) {
            // allow motor to go upwards, but limit at distance above zero
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
            // stop motor by indicating it has reached desired position, making sure not to set
            // position outside of bounds
            int desired = Math.max(0, Math.min(LIFT_MOTOR_MAX, liftMotor.getCurrentPosition()));
            liftMotor.setTargetPosition(desired);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            power = 0;
        }
        liftMotor.setPower(power * liftPowerMultiplier);

        Twigger.getInstance().addLine(".lift()")
                .addData( "position", liftMotor.getCurrentPosition())
                .addData( "target", liftMotor.getTargetPosition())
                .addData( "power", power)
                .addData("liftHigh", isLiftAtHighPoint())
                .addData("liftLow", isLiftAtLowPoint());
        Twigger.getInstance().update();
    }

    public void longSleep(BusyWaitHandler waitHandler, int millis) throws InterruptedException {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.milliseconds() < millis) {
            Thread.sleep(10);
            if (!waitHandler.isActive()) {
                throw new InterruptedException("OpMode Timed Out");
            }
        }
    }
}

