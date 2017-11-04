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

    private static final double JOYSTICK_THRESHOLD = 0.2;

    private boolean liftOverrideButtonDepressed = false;


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

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the robot tilerunner object passing it the OpModes hardwareMap.
        tilerunner.init(hardwareMap, telemetry);
        whackerPosition = tilerunner.jewelWhacker.getPosition();


        Twigger.getInstance()
                .addData("Lift Override", new Func<String>() {
                    @Override
                    public String value() {
                        return String.valueOf(tilerunner.getLiftOverride());
                    }
                });
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

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        Twigger.getInstance().update();

        boolean easy_claw = gamepad1.left_bumper || gamepad1.right_bumper;

        if (!easy_claw) {
            // Normal Mode

            // Wheels (Gamepad 1 Joystick)
            speedLeft = calculateWheelSpeed(speedLeft, -gamepad1.left_stick_y);
            speedRight = calculateWheelSpeed(speedRight, -gamepad1.right_stick_y);

            tilerunner.leftMotor.setPower(speedLeft);
            tilerunner.rightMotor.setPower(speedRight);


            // Claw Motor (Gamepad 1 Triggers)
            if (gamepad1.left_trigger > JOYSTICK_THRESHOLD)
                tilerunner.clawMotor.setPower(gamepad1.left_trigger);
            else if (gamepad1.right_trigger > JOYSTICK_THRESHOLD)
                tilerunner.clawMotor.setPower(-gamepad1.right_trigger);
            else
                tilerunner.clawMotor.setPower(0);

        } else {
            // "Easy" glyph manipulation

            if (gamepad1.left_bumper) {
                // Easy PUT Glyph

                speedLeft = calculateWheelSpeed(speedLeft, -.75);
                speedRight = calculateWheelSpeed(speedRight, -.75);

                tilerunner.leftMotor.setPower(speedLeft);
                tilerunner.rightMotor.setPower(speedRight);


                tilerunner.clawMotor.setPower(1);
            } else {
                // Easy GRAB glyph

                speedLeft = calculateWheelSpeed(speedLeft, .5);
                speedRight = calculateWheelSpeed(speedRight, .5);

                tilerunner.leftMotor.setPower(speedLeft);
                tilerunner.rightMotor.setPower(speedRight);


                tilerunner.clawMotor.setPower(-1);
            }

        }

        // Lift (Gamepad 2 Left Joystick)
        if (Math.abs(gamepad2.left_stick_y) >= JOYSTICK_THRESHOLD)
            tilerunner.setLiftPower(-calculateLiftSpeed(gamepad2.left_stick_y));
        else
            tilerunner.setLiftPower(0);

        // Jewel Whacker (Gamepad 2 Right Joystick)
        if (Math.abs(gamepad2.right_stick_y) >= JOYSTICK_THRESHOLD)
            whackerPosition += WHACKER_CONTROL_SPEED * gamepad1.right_stick_y;
        tilerunner.jewelWhacker.setPosition(whackerPosition);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}

}
