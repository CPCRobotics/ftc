package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.nulls.NullBNO055IMU;
import org.firstinspires.ftc.teamcode.nulls.NullDcMotor;
import org.firstinspires.ftc.teamcode.nulls.NullServo;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

import java.util.Arrays;

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

    private static final double TURN_CORRECTION_THRESHOLD = 2.5; // degrees

    private static final int DISTANCE_REMOVE_GLYPH = 10; // TODO get needed ticks to remove glyph

    DcMotor  clawMotor;
    public DcMotor  leftMotor;
    public DcMotor  rightMotor;
    public DcMotor motorPair;
    DcMotor liftMotor;

    private boolean liftOverride = false;

    private static final int LIFT_LIMIT_LOWER = 0;
    private static final int LIFT_LIMIT_UPPER = 100;

    Servo jewelWhacker;

    BNO055IMU imu;

    private ElapsedTime period  = new ElapsedTime();

    private enum Direction {
        CLOCKWISE {
            public double distanceDegrees(double start, double end) {
                double dist = start-end;
                return (dist >= 0) ? dist : dist + 360;
            }
        },
        COUNTERCLOCKWISE {
            public double distanceDegrees(double start, double end) {
                double dist = end-start;
                return (dist >= 0) ? dist : dist + 360;
            }
        };

        public abstract double distanceDegrees(double start, double end);

        public static Direction fromPower(double power) {
            if (power > 0) {
                return CLOCKWISE;
            } else {
                return COUNTERCLOCKWISE;
            }
        }
    }

    DcMotor createDcMotor(HardwareMap hardwareMap, String motorName){
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
    public void init( HardwareMap hardwareMap, Telemetry telemetry )
    {

        Twigger.getInstance().setTelemetry(telemetry);

        // Define and Initialize Motors
        leftMotor   = createDcMotor(hardwareMap, "left_drive");
        rightMotor  = createDcMotor(hardwareMap, "right_drive");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // Create a motor pair when manipulating both leftMotor and rightMotor
        motorPair = new DCMotorGroup(Arrays.asList(leftMotor, rightMotor));

        liftMotor = createDcMotor(hardwareMap,"lift");
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawMotor = createDcMotor(hardwareMap, "claw");

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
            Twigger.getInstance().sendOnce("Initializing IMU");

            if (!imu.initialize(parameters))
                throw new IllegalArgumentException();

            Twigger.getInstance().sendOnce("Initialized IMU");
        } catch (IllegalArgumentException e) {
            imu = new NullBNO055IMU();
            Twigger.getInstance().sendOnce("WARN: IMU Sensor Missing");
        }

        try {
            jewelWhacker = hardwareMap.servo.get("whacker");
            jewelWhacker.setPosition(1);
        } catch (IllegalArgumentException e) {
            jewelWhacker = new NullServo();
            Twigger.getInstance().sendOnce("WARN: Jewel Whacker Missing");
        }

    }

    double getHeading() {
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

    public void turn(BusyWaitHandler waitHandler, double power, double destinationDegrees)
            throws InterruptedException {

        // Get the sign (-1 or 1) from both directionPower and destinationDegrees
        final double directionSign = Math.signum(power) * Math.signum(destinationDegrees);
        Direction direction = Direction.fromPower(directionSign);

        // Make directionPower and destinationDegrees positive
        power = Math.abs(power);
        destinationDegrees = Math.abs(destinationDegrees);

        final double startHeading = getHeading();

        motorPair.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPair.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPower(power);
        rightMotor.setPower(-power);

        double delta = direction.distanceDegrees(startHeading, getHeading());
        while ((delta < destinationDegrees || delta > 300) && waitHandler.isActive()) {

            double currentPower = power * calculateSpeed(destinationDegrees - delta, THRESHOLD_HEADING);

            currentPower = Math.max(MOTOR_DEADZONE, currentPower);
            currentPower *= directionSign;

            leftMotor.setPower(currentPower);
            rightMotor.setPower(-currentPower);

            delta = direction.distanceDegrees(startHeading, getHeading());

            Twigger.getInstance().addLine(".turn()")
                    .addData("power", currentPower)
                    .addData("delta", delta)
                    .addData("leftpos", leftMotor.getCurrentPosition())
                    .addData("rightpos", rightMotor.getCurrentPosition());

            Thread.sleep(1);
        }

        motorPair.setPower(0);
        motorPair.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Thread.sleep(100);

        // If we have overshot over 5°, correct it at half the speed.
        double overshoot = delta - destinationDegrees;
        if (overshoot > TURN_CORRECTION_THRESHOLD)
            turn(waitHandler, -power/5, overshoot);

        Twigger.getInstance()
                .update()
                .remove(".turn()");
    }

    void moveClaw(BusyWaitHandler waitHandler, int distanceTicks, double power) throws InterruptedException {
        // Stop claw motor and program it to run `distanceTicks` ticks at `power` speed.
        clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawMotor.setTargetPosition(distanceTicks);
        clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawMotor.setPower(power);

        // Wait until either the waitHandler says it's time to stop or the claw motor goes to its
        //      destination
        //noinspection StatementWithEmptyBody
        while (clawMotor.isBusy() && waitHandler.isActive())
            Thread.sleep(10);
    }

    public void removeGlyph(BusyWaitHandler waitHandler, double power) throws InterruptedException {
        moveClaw(waitHandler, DISTANCE_REMOVE_GLYPH, power);
    }

    public void activateJewelWhacker() throws InterruptedException {
        jewelWhacker.setPosition(.75);
        Thread.sleep(750); // Let the tape straighten out
        jewelWhacker.setPosition(0);
    }

    public void retractJewelWhacker() {
        jewelWhacker.setPosition(1);
    }

    /**
     * Toggles whether or not to override the lift
     *
     * @return the new lift status
     */
    public boolean toggleLiftOverride() {
        liftOverride = !liftOverride;
        Twigger.getInstance()
                .sendOnce("Life Override Toggled: " + liftOverride);
        return liftOverride;
    }

    public boolean getLiftOverride() {
        return liftOverride;
    }

    /**
     * Sets the lift power, preventing it to go above its limit.
     * @param power
     * @return true if lift is within limits
     */
    public boolean setLiftPower(double power) {
        // Stop if trying to go below lower limit
        if (power < 0 && liftMotor.getCurrentPosition() <= LIFT_LIMIT_LOWER) {
            liftMotor.setPower(0);
            return false;
        }

        // Stop if trying to go above lower limit
        if (power > 0 && liftMotor.getCurrentPosition() >= LIFT_LIMIT_UPPER) {
            liftMotor.setPower(0);
            return false;
        }

        liftMotor.setPower(power);
        return true;
    }
}

