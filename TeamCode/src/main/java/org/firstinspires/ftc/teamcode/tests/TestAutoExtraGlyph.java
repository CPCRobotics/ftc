package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.ExtraGlyphPhase;
import org.firstinspires.ftc.teamcode.opmodes.EasyLinearOpmode;

@Autonomous(name="Test ExtraGlyph", group="TestConcept")
public class TestAutoExtraGlyph extends EasyLinearOpmode {
    @Override
    protected void startAutonomous() throws InterruptedException {
        new ExtraGlyphPhase(tilerunner, this).grabExtraGlyph();
    }
}
