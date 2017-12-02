/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.twigger.Twigger;


@TeleOp(name = "Competition TeleOp", group = "Iterative Opmode")
public class TankDrive extends OpMode {

    private final static double SPEED_GAIN = 3;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private Tilerunner tilerunner = new Tilerunner();

    private double whackerPosition = 0;
    private static final double WHACKER_CONTROL_SPEED = 0.05;

    private double speedLeft = 0;
    private double speedRight = 0;

    private boolean easyModeTriggered = false;
    private ElapsedTime timeSinceEasyModeTriggered = new ElapsedTime();

    private static final double JOYSTICK_THRESHOLD = 0.2;

    private long gamepad2Timestamp;
    private ElapsedTime gamepad2Time = new ElapsedTime();


    private double calculateWheelSpeed(double currentSpeed, double joystickPower) {
        // Make moving the robot more sensitive when the joystick is closer to 0
        double targetPower = joystickPower * Math.abs(joystickPower);

        // Avoid "deadzone" where the motor doesn't have enough power to move the robot
        targetPower = targetPower * (1 - Tilerunner.MOTOR_DEADZONE) + Tilerunner.MOTOR_DEADZONE * Math.signum(targetPower);

        // Slowly ramp up speed to full speed
        return (targetPower - currentSpeed) / SPEED_GAIN + currentSpeed;
    }

    private double calculateLiftSpeed(double joystickPower) {
        return (joystickPower * joystickPower) * Math.signum(joystickPower);
    }

    private static double withinRange(double min, double max, double x) {
        return Math.min(max, Math.max(min, x));
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the robot tilerunner object passing it the OpModes hardwareMap.
        tilerunner.init(hardwareMap, telemetry);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    private boolean gamepad2Changed() {
        long timestamp = gamepad2.timestamp;
        boolean gamepadChanged = timestamp != gamepad2Timestamp;
        gamepad2Timestamp = timestamp;

        return gamepadChanged;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        Twigger.getInstance().update();

        // Show at the beginning in case it should be overridden
        tilerunner.displayStatus();


        if (gamepad1.left_bumper) {
            // Easy PUT Glyph
            if (!easyModeTriggered)
                timeSinceEasyModeTriggered.reset();
            easyModeTriggered = true;

            if (timeSinceEasyModeTriggered.seconds() >= 0.5) {
                speedLeft = calculateWheelSpeed(speedLeft, -.75);
                speedRight = calculateWheelSpeed(speedRight, -.75);

                tilerunner.leftMotor.setPower(speedLeft);
                tilerunner.rightMotor.setPower(speedRight);
            } else {
                tilerunner.motorPair.setPower(0);
            }


            tilerunner.ejectGlyph(1);
        } else if (gamepad1.right_bumper) {
            // Easy GRAB glyph

            speedLeft = calculateWheelSpeed(speedLeft, .5);
            speedRight = calculateWheelSpeed(speedRight, .5);

            tilerunner.leftMotor.setPower(speedLeft);
            tilerunner.rightMotor.setPower(speedRight);


            tilerunner.grabGlyph(1);
        } else {

            easyModeTriggered = false;

            // Wheels (Gamepad 1 Joystick)

            if (gamepad1.x) {
                tilerunner.leftMotor.setPower(-1);
                tilerunner.rightMotor.setPower(1);
            } else if (gamepad1.b) {
                tilerunner.leftMotor.setPower(1);
                tilerunner.rightMotor.setPower(-1);
            } else {
                speedLeft = calculateWheelSpeed(speedLeft, -gamepad1.left_stick_y);
                speedRight = calculateWheelSpeed(speedRight, -gamepad1.right_stick_y);

                tilerunner.leftMotor.setPower(speedLeft);
                tilerunner.rightMotor.setPower(speedRight);
            }


            // Claw Motor (Gamepad 1 Triggers)
            if (gamepad1.left_trigger > JOYSTICK_THRESHOLD) {
                // put glyph
                tilerunner.ejectGlyph(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > JOYSTICK_THRESHOLD) {
                // grab glyph
                tilerunner.grabGlyph(gamepad1.right_trigger);
            } else {
                tilerunner.grabGlyph(0);
            }
        }

        if (gamepad2Changed()) {
            gamepad2Time.reset();
        }

        // Lift (Gamepad 2 Left Joystick)
        if (Math.abs(gamepad2.left_stick_y) >= JOYSTICK_THRESHOLD)
            tilerunner.setLiftPower(-calculateLiftSpeed(gamepad2.left_stick_y));
        else
            tilerunner.setLiftPower(0);

        // Easy Lift
        // Wait for gamepad2 to be paired for 2 seconds.
        // Since pairing the gamepad requires pushing the B button,
        // it might interfere with the EasyLift functions.
        if (gamepad2Time.seconds() > 2) {
            if (gamepad2.dpad_up)
                tilerunner.changeLiftPosition(true);
            else if (gamepad2.dpad_down)
                tilerunner.changeLiftPosition(false);

            if (gamepad2.y)
                tilerunner.changeLiftPosition(Tilerunner.CryptoboxRow.HIGHEST);
            else if (gamepad2.x)
                tilerunner.changeLiftPosition(Tilerunner.CryptoboxRow.HIGHER);
            else if (gamepad2.b)
                tilerunner.changeLiftPosition(Tilerunner.CryptoboxRow.LOWER);
            else if (gamepad2.a)
                tilerunner.changeLiftPosition(Tilerunner.CryptoboxRow.LOWEST);
        }


        // Jewel Whacker (Gamepad 2 Right Joystick)
        if (Math.abs(gamepad2.right_stick_y) >= JOYSTICK_THRESHOLD) {
            whackerPosition += WHACKER_CONTROL_SPEED * gamepad2.right_stick_y;
            whackerPosition = withinRange(0, 1, whackerPosition);
            tilerunner.jewelWhacker.setPosition(whackerPosition);
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}

}
