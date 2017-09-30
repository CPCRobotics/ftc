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
 * Utitiy class for Vuphoria image recognition
 */

@SuppressWarnings("WeakerAccess")
public class EyesightUtil {

    static VuforiaLocalizer vuforia;
    static VuforiaTrackables relicTrackables;
    static VuforiaTrackable relicTemplate;

    public static void init(OpMode opMode) {
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

    public static void start() {
        relicTrackables.activate();
    }

    public static RelicRecoveryVuMark getPictograph() {
        return RelicRecoveryVuMark.from(relicTemplate);
    }

    public static OpenGLMatrix getPose() {
        return ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
    }
}
