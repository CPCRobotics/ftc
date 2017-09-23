package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by samuel on 9/22/17.
 */

/**
 * Utitiy class for Vuphoria image recognition
 */

class EyesightUtil {

    static VuforiaLocalizer vuforia;
    static VuforiaTrackables relicTrackables;
    static VuforiaTrackable relicTemplate;

    static void init(OpMode opMode) {
        HardwareMap hardwareMap = opMode.hardwareMap;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        /*
         * WARNING! The git repository does not automatically hold the vuphoria license key, and so
         * this will AUTOMATICALLY GIVE A BUILD ERROR!
         *
         * In the TeamCode folder, create a file `res/values/auth.xml` with this code snippet:
         *
         *     <?xml version="1.0" encoding="utf-8"?>
         *     <resources>
         *        <string name="vuphoria">abc/123</string>
         *     </resources>
         *
         * Then replace "abc/123" with the correct vuphoria license key.
         */
        parameters.vuforiaLicenseKey =  hardwareMap.appContext.getString(R.string.vuphoria);

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

    }

    static void start() {
        relicTrackables.activate();
    }

    static RelicRecoveryVuMark getPictograph() {
        return RelicRecoveryVuMark.from(relicTemplate);
    }

    static OpenGLMatrix getPose() {
        return ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
    }
}
