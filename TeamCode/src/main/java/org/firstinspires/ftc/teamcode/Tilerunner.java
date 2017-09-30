package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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
    private static double THRESHOLD_TICKS = Tilerunner.TICKS_PER_REVOLUTION;
    private static double THRESHOLD_HEADING = 30;

    DcMotor  leftMotor;
    DcMotor  rightMotor;
    DcMotor motorPair;

    DcMotor liftMotor;
    BNO055IMU imu;
    Telemetry telemetry;

    private ElapsedTime period  = new ElapsedTime();

    private static double difference(double source, double now) {
        double delta = source - now;

        return delta >= 0 ? delta % 360 : (delta + 360) % 360;
    }

    /* Initialize standard Hardware interfaces */
    void init( HardwareMap hardwareMap, Telemetry telemetry )
    {

        // Define and Initialize Motors
        leftMotor   = hardwareMap.dcMotor.get("left_drive");
        rightMotor  = hardwareMap.dcMotor.get("right_drive");

        try {
            liftMotor = hardwareMap.dcMotor.get("lift");
        } catch (RuntimeException e) {
            telemetry.addData("error", "LIFT MOTOR MISSING: " + e.getMessage());
            liftMotor = new NullDcMotor();
        }

        motorPair = new DCMotorGroup(Arrays.asList(leftMotor, rightMotor));

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        liftMotor.setDirection(DcMotor.Direction.FORWARD); // TODO Ensure this is the correct direction

        // Set all motors to zero power
        motorPair.setPower(0);
        liftMotor.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorPair.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        this.telemetry = telemetry;

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

        // Slow down when the distance to the target is less than 3 * the distance of the wheel
        return Math.min(1, Math.max(0.5, dist / threshold) );
    }

    void move(BusyWaitHandler waitHandler, double inches, double power) {
        int ticks = (int)(Tilerunner.TICKS_PER_REVOLUTION * inches / Tilerunner.WHEEL_CIRCUMFERENCE);

        motorPair.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPair.setTargetPosition( ticks );
        motorPair.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorPair.setPower(power);

        while (leftMotor.isBusy() && waitHandler.isActive()) {
            motorPair.setPower(calculateSpeed(motorPair.getTargetPosition() - motorPair.getCurrentPosition(), THRESHOLD_TICKS));
        }
    }

    void turn(BusyWaitHandler waitHandler, double direction, double degrees) {
        double heading = getHeading();

        motorPair.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(direction);
        rightMotor.setPower(-direction);

        double delta = difference(heading, getHeading());
        while ( delta < degrees && waitHandler.isActive()) {
            double power = direction * calculateSpeed(degrees - delta, THRESHOLD_HEADING);
            telemetry.addData("goal", degrees);
            telemetry.addData("delta", delta);
            leftMotor.setPower(power);
            rightMotor.setPower(-power);

            delta = difference(heading, getHeading());
        }
        motorPair.setPower(0);
    }

    void lift(BusyWaitHandler waitHandler, int distanceTicks, double power) {

        // Stop lift motor and program it to run `distanceTicks` ticks at `power` speed.
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(distanceTicks);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);

        // Wait until either the waitHandler says it's time to stop or the lift motor goes to its
        //      destination
        //noinspection StatementWithEmptyBody
        while (liftMotor.isBusy() && waitHandler.isActive()) {
        }
    }

}

