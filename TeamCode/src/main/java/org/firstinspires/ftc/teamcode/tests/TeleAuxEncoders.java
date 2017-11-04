package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.Tilerunner;
import org.firstinspires.ftc.teamcode.twigger.Twigger;

@TeleOp(name="Auxillary Encoders", group="Test")
public class TeleAuxEncoders extends OpMode {
    Tilerunner tilerunner = new Tilerunner();

    private boolean liftOverrideButtonDepressed = false;

    @Override
    public void init() {
        tilerunner.init(hardwareMap, telemetry);

        Twigger.getInstance()
                .addLine("Pos")
                    .addData("lift", new Func<String>() {
                        @Override
                        public String value() {
                            return String.valueOf(tilerunner.liftMotor.getCurrentPosition());
                        }
                    })
                    .addData("claw", new Func<String>() {
                        @Override
                        public String value() {
                            return String.valueOf(tilerunner.clawMotor.getCurrentPosition());
                        }
                    });
    }

    @Override
    public void loop() {

        // Lift (D-Pad)
        if (gamepad1.dpad_up)
            tilerunner.liftMotor.setPower(1);
        else if (gamepad1.dpad_down)
            tilerunner.liftMotor.setPower(-1);
        else
            tilerunner.liftMotor.setPower(0);


        // Claw Motor (Triggers)
        if (gamepad1.left_trigger > 0.2)
            tilerunner.clawMotor.setPower(-gamepad1.left_trigger);
        else if (gamepad1.right_trigger > 0.2)
            tilerunner.clawMotor.setPower(gamepad1.right_trigger);
        else
            tilerunner.clawMotor.setPower(0);

        // Lift Override (B Button)
        if (gamepad1.b && !liftOverrideButtonDepressed) {
            liftOverrideButtonDepressed = true;
            //tilerunner.toggleLiftOverride();
        } else if (!gamepad1.b) {
            liftOverrideButtonDepressed = false;
        }
    }
}
