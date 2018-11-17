/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TileRunner;


@TeleOp(name="UltrasonicTest", group="test")

public class UltrasonicTest extends LinearOpMode
{
    TileRunner robot = new TileRunner();

    @Override
    public void runOpMode()
    {

        ElapsedTime pulseWidth = new ElapsedTime();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
        waitForStart();
        pulseWidth.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            //send the trigger pulse
            robot.HCTrig.setMode(DigitalChannel.Mode.OUTPUT);

            //reset the timer to get echo
            pulseWidth.reset();
            while(!robot.HCEcho.getState() && opModeIsActive())
            {
                //wait for echo to come back
            }
            robot.HCTrig.setMode(DigitalChannel.Mode.INPUT);
            telemetry.addData("currentState", robot.HCEcho.getState());
            telemetry.addData("Pulse width", pulseWidth.nanoseconds());
            telemetry.update();
            sleep(500);
        }
    }

    /*
                //send the trigger pulse
            robot.HCTrig.setMode(DigitalChannel.Mode.OUTPUT);
            pulseWidth.reset();
            while(pulseWidth.nanoseconds() <= 10 * 1000 && opModeIsActive())
            {
                //wait a bit while the sensor recieves the trigger pulse
            }

            //stop sending the trigger pulse
            robot.HCTrig.setMode(DigitalChannel.Mode.INPUT);
            //reset the timer to get echo
            pulseWidth.reset();
            while(!robot.HCEcho.getState() && opModeIsActive())
            {
                //wait for echo to come back
//                telemetry.addData("Echo is active", robot.HCEcho.getState());
//                telemetry.update();
            }
            telemetry.addData("currentState", robot.HCEcho.getState());
            telemetry.addData("Pulse width", pulseWidth.nanoseconds());
            telemetry.update();
     */
}
