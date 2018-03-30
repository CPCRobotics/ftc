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
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.feature.EasyPutFeature;
import org.firstinspires.ftc.teamcode.opmodes.feature.EasyStackFeature;
import org.firstinspires.ftc.teamcode.opmodes.feature.Feature;
import org.firstinspires.ftc.teamcode.opmodes.feature.TurnFeature;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;
import org.firstinspires.ftc.teamcode.util.ThresholdTrigger;


@TeleOp(name = "Competition TeleOp", group = "Iterative Opmode")
public class TankDrive extends OpMode {

    /* Declare OpMode members. */
    private final ElapsedTime runtime = new ElapsedTime();
    private final Tilerunner tilerunner = new Tilerunner();

    private final Feature easyStack = new EasyStackFeature(tilerunner);
    private final Feature easyPut = new EasyPutFeature(tilerunner);
    private final Feature easyTurn = new TurnFeature(tilerunner);

    private final ThresholdTrigger glyphEject = new ThresholdTrigger();
    private final ThresholdTrigger glyphGrab = new ThresholdTrigger();

    private static double clamp(double val, double min, double max) {
        if (val < min) return min;
        if (val > max) return max;
        return val;
    }

    private static double calculateLiftSpeed(double joystickPower) {
        double result = (joystickPower * joystickPower) * Math.signum(joystickPower);
        return clamp(result * 5, -1, 1);
    }
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the robot tilerunner object passing it the OpModes hardwareMap.
        tilerunner.init(hardwareMap, telemetry, Tilerunner.OpmodeType.TELEOP);
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

        tilerunner.retractJewelWhacker();

        boolean glyphDetected = tilerunner.glyphDetected();
        tilerunner.setLights(glyphDetected, glyphDetected);

        // "Special" function features
        if (easyPut.call(gamepad1.left_bumper)) return;
        if (easyStack.call(gamepad1.right_bumper)) return;

        if (gamepad1.x) {
            easyTurn.call(-1);
            return;
        } else if (gamepad1.b) {
            easyTurn.call(1);
            return;
        }

        tilerunner.setMotors(-gamepad1.left_stick_y, -gamepad1.right_stick_y);



        // Claw Motor (Gamepad 1 Triggers)
        double putVal = glyphEject.get(gamepad1.left_trigger);
        if (putVal > 0) {
            tilerunner.ejectGlyph(putVal);
        } else {
            tilerunner.grabGlyph(glyphGrab.get(gamepad1.right_trigger));
        }


        // Lift (Gamepad 2 Left Joystick)
        tilerunner.setLiftPower(calculateLiftSpeed(-gamepad2.left_stick_y));

        if (gamepad2.dpad_up)
            tilerunner.changeLiftPosition(true);
        else if (gamepad2.dpad_down)
            tilerunner.changeLiftPosition(false);

        // Glyph Holder
        if (gamepad2.y)
            tilerunner.setHolderUp();
        else if (gamepad2.a)
            tilerunner.setHolderDown();

    }

}
